"""Stop/manual/auto supervisor (no rclpy)."""
import enum
import functools
import math
import threading
from typing import Callable, Optional

from .rosio import RosIo


class Mode(enum.IntEnum):
    MANUAL = 0
    AUTO = 1
    STOP = 2


def _noop(ok, msg):
    pass


def _locked(fn):
    @functools.wraps(fn)
    def wrapper(self, *a, **k):
        with self.lock:
            return fn(self, *a, **k)
    return wrapper


class Supervisor:
    def __init__(self, io: RosIo, lift_max: float, timeout_s: float = 0.3,
                 max_v: float = 2.0, max_w: float = 2.5):
        self.lock = threading.RLock()
        self.io = io
        self.lift_max = lift_max
        self.timeout_s = timeout_s
        self.max_v = max_v
        self.max_w = max_w
        self.mode = Mode.STOP
        self.stop_reason = "link lost"
        self.auto_label = ""
        self.on_stop: Optional[Callable[[], None]] = None
        self._last_rx: Optional[float] = None
        self._now = 0.0
        self._estop = False
        self._v = (0.0, 0.0, 0.0)
        self._field = False
        self._jog = [0, 0]

    # -- inputs --------------------------------------------------------
    @_locked
    def on_control(self, now_s: float, c: dict) -> None:
        self._now = now_s
        self._last_rx = now_s
        self._estop = bool(c["estop"])
        self._v = (float(c["vx"]), float(c["vy"]), float(c["wz"]))
        self._field = bool(c["field_frame"])
        jogs = [int(c["jog_left"]), int(c["jog_right"])]
        if self._estop:
            if self.mode != Mode.STOP:
                self._enter_stop("estop")
            else:
                self.stop_reason = "estop"
        if self.mode == Mode.STOP:
            self._jog = jogs  # held keys do not start anything after release
            return
        for g in (0, 1):
            if jogs[g] != self._jog[g]:
                if jogs[g] > 0:
                    self.io.lift_to(g, self.lift_max, _noop)
                elif jogs[g] < 0:
                    self.io.lift_to(g, 0.0, _noop)
                else:
                    self.io.lift_cancel(g)
        self._jog = jogs

    @_locked
    def tick(self, now_s: float, yaw: Optional[float]) -> None:
        self._now = now_s
        if self.mode != Mode.STOP and self._timed_out(now_s):
            self._enter_stop("link lost")
        if self.mode == Mode.STOP:
            self.io.publish_cmd_vel(0.0, 0.0, 0.0)
        elif self.mode == Mode.MANUAL:
            vx, vy, wz = self._v
            if self._field and yaw is not None:
                c, s = math.cos(yaw), math.sin(yaw)
                vx, vy = c * vx + s * vy, -s * vx + c * vy
            n = math.hypot(vx, vy)
            if n > self.max_v:
                vx, vy = vx * self.max_v / n, vy * self.max_v / n
            wz = max(-self.max_w, min(self.max_w, wz))
            self.io.publish_cmd_vel(vx, vy, wz)

    # -- transitions ---------------------------------------------------
    @_locked
    def release(self):
        if self._estop:
            return False, "estop is still pressed"
        if self._last_rx is None or self._timed_out(self._now):
            return False, "no control received"
        if self.mode == Mode.STOP:
            self.mode = Mode.MANUAL
            self.stop_reason = ""
        return True, ""

    @_locked
    def enter_auto(self, label: str) -> None:
        if self.mode != Mode.STOP:
            self.mode = Mode.AUTO
            self.auto_label = label

    @_locked
    def leave_auto(self) -> None:
        if self.mode == Mode.AUTO:
            self.mode = Mode.MANUAL
            self.auto_label = ""

    @_locked
    def cancel_to_manual(self) -> None:
        self.leave_auto()

    # -- internals -----------------------------------------------------
    def _timed_out(self, now_s: float) -> bool:
        return self._last_rx is None or now_s - self._last_rx > self.timeout_s

    @_locked
    def _enter_stop(self, reason: str) -> None:
        self.mode = Mode.STOP
        self.stop_reason = reason
        self.auto_label = ""
        io = self.io
        io.cancel_task()
        io.stop_path(_noop)
        io.set_tracker(False, _noop)
        for g in (0, 1):
            io.lift_cancel(g)
        for g in (0, 1):
            io.set_suction(g, False, _noop)
        if self.on_stop:
            self.on_stop()
