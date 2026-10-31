"""Stop/manual/auto supervisor (no rclpy)."""
import enum
import functools
import math
import sys
import threading
import traceback
from typing import Callable, Optional

from .harite import HariteDriver
from .rosio import RosIo


class Mode(enum.IntEnum):
    MANUAL = 0
    AUTO = 1
    STOP = 2


def _noop(ok, msg):
    pass


def _fin(v) -> float:
    try:
        v = float(v)
    except (TypeError, ValueError):
        return 0.0
    return v if math.isfinite(v) else 0.0


def _locked(fn):
    @functools.wraps(fn)
    def wrapper(self, *a, **k):
        with self.lock:
            return fn(self, *a, **k)
    return wrapper


class Supervisor:
    def __init__(self, io: RosIo, lift_max: float, timeout_s: float = 0.3,
                 max_v: float = 2.0, max_w: float = 2.5,
                 harite_speed: Optional[float] = None, harite_reversed: bool = False):
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
        # Harite (no suction, speed-controlled lift): None speed = the robot has none, never touched.
        self.harite = (HariteDriver(io, self.lock, harite_speed, harite_reversed)
                       if harite_speed is not None else None)

    # -- inputs --------------------------------------------------------
    @_locked
    def on_control(self, now_s: float, c: dict) -> None:
        self._now = now_s
        self._last_rx = now_s
        self._estop = bool(c["estop"])
        self._v = (_fin(c["vx"]), _fin(c["vy"]), _fin(c["wz"]))
        self._field = bool(c["field_frame"])
        jogs = [int(c["jog_left"]), int(c["jog_right"])]
        if self._estop:
            if self.mode != Mode.STOP:
                self._enter_stop("estop")
            else:
                self.stop_reason = "estop"
        if self.harite is not None:
            self.harite.on_control(c.get("jog_harite", 0), self.mode == Mode.MANUAL)
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
        if self.harite is not None:
            self.harite.tick(now_s, self.mode == Mode.MANUAL, self.mode == Mode.STOP)

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
            self._v = (0.0, 0.0, 0.0)  # drive only on a CONTROL received after the release
            if self.harite is not None:
                self.harite.release(self._now)
        return True, ""

    @_locked
    def enter_auto(self, label: str) -> None:
        if self.mode != Mode.STOP:
            self.mode = Mode.AUTO
            self.auto_label = label
            if self.harite is not None:
                self.harite.auto()

    @_locked
    def leave_auto(self) -> None:
        if self.mode == Mode.AUTO:
            self.mode = Mode.MANUAL
            self.auto_label = ""

    @_locked
    def cancel_to_manual(self) -> None:
        self.leave_auto()

    @_locked
    def link_lost(self) -> None:
        """Enter STOP with reason "link lost" (no-op if already stopped)."""
        if self.mode != Mode.STOP:
            self._enter_stop("link lost")

    # -- internals -----------------------------------------------------
    def _timed_out(self, now_s: float) -> bool:
        return self._last_rx is None or now_s - self._last_rx > self.timeout_s

    @_locked
    def _enter_stop(self, reason: str) -> None:
        self.mode = Mode.STOP
        self.stop_reason = reason
        self.auto_label = ""
        io = self.io
        steps = []
        if self.harite is not None:  # first: 0, then disable (it cannot be left moving)
            steps.append(("harite stop", lambda: self.harite.stop(self._now)))
        steps += [("cancel_task", io.cancel_task),
                 ("stop_path", lambda: io.stop_path(_noop)),
                 ("set_tracker", lambda: io.set_tracker(False, _noop))]
        steps += [(f"lift_cancel {g}", lambda g=g: io.lift_cancel(g)) for g in (0, 1)]
        steps += [(f"set_suction {g}", lambda g=g: io.set_suction(g, False, _noop)) for g in (0, 1)]
        for name, step in steps:  # each step on its own: suction-off must always run
            try:
                step()
            except Exception:
                print(f"stop: {name} failed", file=sys.stderr)
                traceback.print_exc()
        if self.on_stop:
            self.on_stop()
