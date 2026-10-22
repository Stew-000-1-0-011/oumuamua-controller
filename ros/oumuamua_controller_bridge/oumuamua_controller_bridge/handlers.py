"""Request handlers for the phone's reliable messages (no rclpy)."""
import threading
from typing import Callable, Optional

from . import protocol as P
from .supervisor import Mode, Supervisor


def _noop(ok, msg):
    pass


class Handlers:
    def __init__(self, io, sup: Supervisor, shells, complete, send: Callable[[list], None],
                 config: dict):
        self.io = io
        self.sup = sup
        self.shells = shells
        self.complete = complete
        self.send = send
        self.config = config
        self.log_level = 20
        self.current_task: Optional[tuple] = None  # (kind, step, (gripper, block, column, row))
        self.current_path: Optional[tuple] = None  # (name, active)
        self._epoch = 0
        sup.on_stop = self._on_stop
        self._table = {
            P.RELEASE_STOP: self._release_stop,
            P.SET_SUCTION: self._set_suction,
            P.LIFT_TO: self._lift_to,
            P.LIFT_ZERO: self._lift_zero,
            P.RUN_TASK: self._run_task,
            P.CANCEL: self._cancel,
            P.START_PATH: self._start_path,
            P.SET_TRACKER: self._set_tracker,
            P.SET_POSE: self._set_pose,
            P.TASK_RESET: self._task_reset,
            P.LOG_LEVEL: self._log_level,
            P.SHELL_OPEN: self._shell_open,
            P.SHELL_INPUT: self._shell_input,
            P.SHELL_COMPLETE: self._shell_complete,
            P.SHELL_CLOSE: self._shell_close,
            P.SHELL_RENAME: self._shell_rename,
        }

    def _invalidate(self):
        """Forget running work; late callbacks of older work are stale."""
        self._epoch += 1
        self.current_task = None
        self.current_path = None

    def _on_stop(self):
        self._invalidate()

    def _result(self, req, ok, message=""):
        self.send([P.RESULT, req, bool(ok), message])

    def handle(self, msg: list) -> None:
        if not msg or msg[0] not in self._table:
            return
        req = msg[1] if len(msg) > 1 and isinstance(msg[1], int) else None
        try:
            self._table[msg[0]](req, msg)
        except (IndexError, TypeError, ValueError, KeyError) as e:
            if req is not None:
                self._result(req, False, f"bad request: {e}")

    # -- helpers -------------------------------------------------------
    @staticmethod
    def _targets(gripper):
        if gripper == 2:
            return [0, 1]
        if gripper in (0, 1):
            return [gripper]
        raise ValueError(f"bad gripper {gripper}")

    def _each(self, req, gripper, call):
        targets = self._targets(gripper)
        lock = threading.Lock()
        results = {}

        def make_done(g):
            def done(ok, message):
                with lock:
                    results[g] = (ok, message)
                    if len(results) < len(targets):
                        return
                fails = [f"gripper {t}: {results[t][1]}" if len(targets) > 1 else results[t][1]
                         for t in targets if not results[t][0]]
                self._result(req, not fails, "; ".join(fails))
            return done

        for g in targets:
            call(g, make_done(g))

    def _simple(self, req, call):
        def done(ok, message):
            self._result(req, ok, message)
        call(done)

    # -- handlers ------------------------------------------------------
    def _release_stop(self, req, m):
        ok, message = self.sup.release()
        self._result(req, ok, message)

    def _set_suction(self, req, m):
        on = bool(m[3])
        self._each(req, m[2], lambda g, d: self.io.set_suction(g, on, d))

    def _lift_to(self, req, m):
        h = float(m[3])
        self._targets(m[2])
        if not 0.0 <= h <= float(self.config["lift_max"]):
            self._result(req, False, f"height out of range (0..{self.config['lift_max']})")
            return
        self._each(req, m[2], lambda g, d: self.io.lift_to(g, h, d))

    def _lift_zero(self, req, m):
        self._each(req, m[2], lambda g, d: self.io.lift_zero(g, d))

    def _busy(self):
        return self.current_task is not None or self.current_path is not None

    def _run_task(self, req, m):
        kind, gripper, block, column, row = (int(x) for x in m[2:7])
        with self.sup.lock:
            if self._busy():
                self._result(req, False, "busy")
                return
            if self.sup.mode == Mode.STOP:
                self._result(req, False, f"stopped: {self.sup.stop_reason}")
                return
            args = (gripper, block, column, row)
            epoch = self._epoch
            self.current_task = (kind, "", args)
            self.sup.enter_auto("task")
            self._result(req, True, "")

        def step(text):
            with self.sup.lock:
                if epoch == self._epoch and self.current_task is not None:
                    self.current_task = (kind, text, args)

        def done(ok, message):
            with self.sup.lock:
                if epoch != self._epoch:
                    return  # stopped or cancelled meanwhile: stale
                self.current_task = None
                self.send([P.TASK_DONE, kind, bool(ok), message])
                self.sup.leave_auto()

        self.io.run_task(kind, gripper, block, column, row, step, done)

    def _cancel(self, req, m):
        with self.sup.lock:
            self._invalidate()
            self.io.cancel_task()
            self.io.stop_path(_noop)
            self.io.set_tracker(False, _noop)
            self.sup.cancel_to_manual()
            self._result(req, True, "")

    def _start_path(self, req, m):
        name = str(m[2])
        with self.sup.lock:
            if self._busy():
                self._result(req, False, "busy")
                return
            if self.sup.mode == Mode.STOP:
                self._result(req, False, f"stopped: {self.sup.stop_reason}")
                return
            epoch = self._epoch
            self.current_path = (name, False)

        def fail(message):
            self.current_path = None
            self._result(req, False, message)

        def stale():
            return epoch != self._epoch or self.sup.mode == Mode.STOP

        def started(ok, message):
            with self.sup.lock:
                if stale():
                    if ok:  # the path may be running: undo it
                        self.io.stop_path(_noop)
                        self.io.set_tracker(False, _noop)
                    self._result(req, False, "cancelled")
                    return
                if not ok:
                    self.io.set_tracker(False, _noop)
                    fail(message)
                    return
                self.sup.enter_auto(name)
                self.current_path = (name, True)
                self._result(req, True, "")

        def tracker(ok, message):
            with self.sup.lock:
                if stale():
                    if ok:
                        self.io.set_tracker(False, _noop)
                    self._result(req, False, "cancelled")
                    return
                if not ok:
                    fail(message)
                    return
                self.io.start_path(name, started)

        self.io.set_tracker(True, tracker)

    def _set_tracker(self, req, m):
        on = bool(m[2])
        self._simple(req, lambda d: self.io.set_tracker(on, d))

    def _set_pose(self, req, m):
        self.io.set_pose(float(m[2]), float(m[3]), float(m[4]))
        self._result(req, True, "")

    def _task_reset(self, req, m):
        self._simple(req, lambda d: self.io.task_reset(d))

    def _log_level(self, req, m):
        self.log_level = int(m[2])
        self._result(req, True, "")

    def _shell_open(self, req, m):
        tab = m[2]
        ok, message = self.shells.open(tab)
        if ok:
            self.send([P.SHELL_OPENED, tab, self.shells.replay(tab)])
        self._result(req, ok, message)

    def _shell_input(self, req, m):
        ok, message = self.shells.write(m[2], bytes(m[3]))
        self._result(req, ok, message)

    def _shell_complete(self, req, m):
        tab, line, cursor = m[2], m[3], int(m[4])
        start, cands = self.complete(line, cursor, self.shells.cwd(tab))
        self.send([P.SHELL_COMPLETION, req, tab, int(start), list(cands)])
        self._result(req, True, "")

    def _shell_close(self, req, m):
        ok, message = self.shells.close(m[2])
        self._result(req, ok, message)

    def _shell_rename(self, req, m):
        ok, message = self.shells.rename(m[2], m[3])
        self._result(req, ok, message)
