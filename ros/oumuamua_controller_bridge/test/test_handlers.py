from fake_rosio import FakeRosIo
from oumuamua_controller_bridge import protocol as P
from oumuamua_controller_bridge.handlers import Handlers
from oumuamua_controller_bridge.supervisor import Supervisor, Mode


class FakeShells:
    def __init__(self):
        self.calls = []
        self.tabs = set()

    def open(self, tab):
        self.calls.append(("open", tab))
        if tab in self.tabs:
            return False, "exists"
        self.tabs.add(tab)
        return True, ""

    def write(self, tab, data):
        self.calls.append(("write", tab, data))
        return (tab in self.tabs), ("" if tab in self.tabs else "no such tab")

    def close(self, tab):
        self.calls.append(("close", tab))
        return True, ""

    def rename(self, tab, new):
        self.calls.append(("rename", tab, new))
        return True, ""

    def replay(self, tab):
        return b"hist"

    def cwd(self, tab):
        return "/home/x"


def ctl(**kw):
    c = dict(session=1, seq=1, t_ms=0, vx=0.0, vy=0.0, wz=0.0, field_frame=False,
             jog_left=0, jog_right=0, estop=False)
    c.update(kw)
    return c


def make(auto_done=True):
    io = FakeRosIo(auto_done)
    sup = Supervisor(io, lift_max=0.5)
    sup.on_control(0.0, ctl())
    sup.release()
    sent = []
    completes = []

    def complete(line, cursor, cwd):
        completes.append((line, cursor, cwd))
        return 3, ["abc", "abd"]

    h = Handlers(io, sup, FakeShells(), complete, sent.append, {"lift_max": 0.5},
                 spawn=lambda f: f())
    io.calls.clear()
    return io, sup, h, sent, completes


def results(sent):
    return [m for m in sent if m[0] == P.RESULT]


def test_release_stop():
    io, sup, h, sent, _ = make()
    sup.on_control(1.0, ctl(estop=True))
    h.handle([P.RELEASE_STOP, 1])
    assert sent[-1][:3] == [P.RESULT, 1, False]
    sup.on_control(1.02, ctl())
    h.handle([P.RELEASE_STOP, 2])
    assert sent[-1] == [P.RESULT, 2, True, ""]
    assert sup.mode == Mode.MANUAL


def test_suction_both_and_failure():
    io, sup, h, sent, _ = make()
    h.handle([P.SET_SUCTION, 1, 2, True])
    assert io.calls == [("set_suction", 0, True), ("set_suction", 1, True)]
    assert sent == [[P.RESULT, 1, True, ""]]
    io.fail[("set_suction", 1)] = "boom"
    sent.clear()
    h.handle([P.SET_SUCTION, 2, 2, False])
    assert sent[0][:3] == [P.RESULT, 2, False] and "boom" in sent[0][3]
    sent.clear()
    h.handle([P.SET_SUCTION, 3, 0, True])
    assert io.calls[-1] == ("set_suction", 0, True) and sent[0][2] is True
    h.handle([P.SET_SUCTION, 4, 5, True])
    assert sent[-1][2] is False


def test_lift_to_above_max_rejected():
    io, sup, h, sent, _ = make()
    h.handle([P.LIFT_TO, 1, 0, 0.6])
    assert io.calls == [] and sent[-1][:3] == [P.RESULT, 1, False]
    h.handle([P.LIFT_TO, 2, 1, 0.5])
    assert io.calls == [("lift_to", 1, 0.5)] and sent[-1][2] is True
    h.handle([P.LIFT_ZERO, 3, 2])
    assert io.calls[-2:] == [("lift_zero", 0), ("lift_zero", 1)]


def run_task(io, h, req=1):
    """RUN_TASK, then let the tracker enable succeed (auto_done=False fakes)."""
    h.handle([P.RUN_TASK, req, 2, 0, 1, 2, 3])
    name, done = io.pending.pop(0)
    assert name == "set_tracker"
    done(True, "")


def test_run_task_enters_auto_and_done_returns_to_manual():
    io, sup, h, sent, _ = make(auto_done=False)
    h.handle([P.RUN_TASK, 1, 2, 0, 1, 2, 3])
    assert io.calls == [("set_tracker", True)]  # tracking first: task_commander drives
    assert sent == [[P.RESULT, 1, True, ""]]
    assert sup.mode == Mode.AUTO
    io.pending.pop(0)[1](True, "")
    assert io.calls == [("set_tracker", True), ("run_task", 2, 0, 1, 2, 3)]
    assert h.current_task == (2, "", (0, 1, 2, 3))
    io.task_step("lift up")
    assert h.current_task == (2, "lift up", (0, 1, 2, 3))
    io.task_done(True, "ok")
    assert sent[-1] == [P.TASK_DONE, 2, True, "ok"]
    assert io.calls[-1] == ("set_tracker", False)
    assert h.current_task is None and sup.mode == Mode.MANUAL


def test_run_task_tracker_failure_ends_task():
    io, sup, h, sent, _ = make(auto_done=False)
    h.handle([P.RUN_TASK, 1, 2, 0, 1, 2, 3])
    io.pending.pop(0)[1](False, "no service")
    assert ("run_task", 2, 0, 1, 2, 3) not in io.calls
    assert sent[-1] == [P.TASK_DONE, 2, False, "tracker: no service"]
    assert h.current_task is None and sup.mode == Mode.MANUAL


def test_estop_while_task_tracker_pending_does_not_start_task():
    io, sup, h, sent, _ = make(auto_done=False)
    h.handle([P.RUN_TASK, 1, 2, 0, 1, 2, 3])
    sup.on_control(1.0, ctl(estop=True))
    io.calls.clear()
    io.pending.pop(0)[1](True, "")  # late enable
    assert not any(c[0] == "run_task" for c in io.calls)
    assert ("set_tracker", False) in io.calls
    assert sup.mode == Mode.STOP and h.current_task is None


def test_busy_rejects_second_task():
    io, sup, h, sent, _ = make(auto_done=False)
    run_task(io, h)
    sent.clear()
    h.handle([P.RUN_TASK, 2, 2, 0, 1, 2, 3])
    assert sent == [[P.RESULT, 2, False, "busy"]]
    h.handle([P.START_PATH, 3, "p"])
    assert sent[-1] == [P.RESULT, 3, False, "busy"]
    assert [c for c in io.calls if c[0] == "run_task"] == [("run_task", 2, 0, 1, 2, 3)]


def test_start_path_and_cancel():
    io, sup, h, sent, _ = make()
    h.handle([P.START_PATH, 1, "slalom"])
    assert io.calls == [("set_tracker", True), ("start_path", "slalom")]
    assert sent == [[P.RESULT, 1, True, ""]]
    assert sup.mode == Mode.AUTO and h.current_path == ("slalom", True)
    io.calls.clear()
    h.handle([P.CANCEL, 2])
    assert io.calls == [("cancel_task",), ("stop_path",), ("set_tracker", False)]
    assert sup.mode == Mode.MANUAL and h.current_path is None
    assert sent[-1] == [P.RESULT, 2, True, ""]


def test_start_path_failure():
    io, sup, h, sent, _ = make()
    io.fail["start_path"] = "no such path"
    h.handle([P.START_PATH, 1, "zzz"])
    assert sent[-1][:3] == [P.RESULT, 1, False] and "no such path" in sent[-1][3]
    assert sup.mode == Mode.MANUAL and h.current_path is None


def test_tracker_pose_reset_loglevel():
    io, sup, h, sent, _ = make()
    h.handle([P.SET_TRACKER, 1, True])
    h.handle([P.SET_POSE, 2, 1.0, 2.0, 0.5])
    h.handle([P.TASK_RESET, 3])
    assert io.calls == [("set_tracker", True), ("set_pose", 1.0, 2.0, 0.5), ("task_reset",)]
    assert all(r[2] for r in results(sent)) and len(sent) == 3
    assert h.log_level == 20
    h.handle([P.LOG_LEVEL, 4, 30])
    assert h.log_level == 30 and sent[-1] == [P.RESULT, 4, True, ""]


def test_shell_flow():
    io, sup, h, sent, completes = make()
    h.handle([P.SHELL_OPEN, 1, "a"])
    assert sent == [[P.SHELL_OPENED, "a", b"hist"], [P.RESULT, 1, True, ""]]
    sent.clear()
    h.handle([P.SHELL_OPEN, 2, "a"])
    assert sent == [[P.RESULT, 2, False, "exists"]]
    sent.clear()
    h.handle([P.SHELL_INPUT, 3, "a", b"ls\n"])
    assert sent == [[P.RESULT, 3, True, ""]]
    sent.clear()
    h.handle([P.SHELL_COMPLETE, 4, "a", "ls ab", 5])
    assert completes == [("ls ab", 5, "/home/x")]
    assert sent == [[P.SHELL_COMPLETION, 4, "a", 3, ["abc", "abd"]], [P.RESULT, 4, True, ""]]
    sent.clear()
    h.handle([P.SHELL_RENAME, 5, "a", "b"])
    h.handle([P.SHELL_CLOSE, 6, "b"])
    assert [r[2] for r in sent if r[0] == P.RESULT] == [True, True]
    assert [m for m in sent if m[0] == P.SHELL_CLOSED] == [[P.SHELL_CLOSED, "b", "closed"]]


def test_malformed_gets_failed_result():
    io, sup, h, sent, _ = make()
    h.handle([P.LIFT_TO, 1])
    assert sent[-1][:3] == [P.RESULT, 1, False]
    h.handle([999, 1])
    h.handle([])


def test_estop_while_tracker_pending_blocks_start_path():
    io, sup, h, sent, _ = make(auto_done=False)
    h.handle([P.START_PATH, 1, "p"])
    assert io.calls == [("set_tracker", True)]
    sup.on_control(1.0, ctl(estop=True))
    assert sup.mode == Mode.STOP
    io.calls.clear()
    name, done = io.pending[0]
    assert name == "set_tracker"
    done(True, "")
    assert ("start_path", "p") not in io.calls
    assert ("set_tracker", False) in io.calls
    assert sent[-1] == [P.RESULT, 1, False, "cancelled"]
    assert sup.mode == Mode.STOP and h.current_path is None


def test_estop_while_start_path_pending_undoes_path():
    io, sup, h, sent, _ = make(auto_done=False)
    h.handle([P.START_PATH, 1, "p"])
    io.pending.pop(0)[1](True, "")
    assert io.calls[-1] == ("start_path", "p")
    sup.on_control(1.0, ctl(estop=True))
    io.calls.clear()
    [d for n, d in io.pending if n == "start_path"][0](True, "")  # late completion
    assert ("stop_path",) in io.calls and ("set_tracker", False) in io.calls
    assert sup.mode == Mode.STOP and h.current_path is None


def test_cancel_while_start_path_pending():
    io, sup, h, sent, _ = make(auto_done=False)
    h.handle([P.START_PATH, 1, "p"])
    io.pending.pop(0)[1](True, "")
    start_done = [d for n, d in io.pending if n == "start_path"][0]
    h.handle([P.CANCEL, 2])
    start_done(True, "")
    assert sup.mode == Mode.MANUAL and h.current_path is None
    assert [m for m in sent if m[1] == 1][-1] == [P.RESULT, 1, False, "cancelled"]


def test_late_task_done_after_estop_release_is_ignored():
    io, sup, h, sent, _ = make(auto_done=False)
    run_task(io, h)
    old_done, old_step = io.task_done, io.task_step
    sup.on_control(1.0, ctl(estop=True))
    assert h.current_task is None
    sup.on_control(1.02, ctl())
    assert sup.release() == (True, "")
    sent.clear()
    old_step("late")
    old_done(False, "cancelled")
    assert sent == [] and sup.mode == Mode.MANUAL and h.current_task is None


def test_cancel_clears_current_task_immediately():
    io, sup, h, sent, _ = make(auto_done=False)
    h.handle([P.RUN_TASK, 1, 2, 0, 1, 2, 3])
    h.handle([P.CANCEL, 2])
    assert h.current_task is None
    h.handle([P.RUN_TASK, 3, 2, 0, 1, 2, 3])
    assert sent[-1] == [P.RESULT, 3, True, ""]


def test_estop_during_run_task_call_cancels_started_task():
    io, sup, h, sent, _ = make(auto_done=False)
    io.on_run_task = lambda: sup.on_control(1.0, ctl(estop=True))
    run_task(io, h)
    names = [c[0] for c in io.calls]
    assert names.index("run_task") < len(names) - 1 - names[::-1].index("cancel_task")
    assert sup.mode == Mode.STOP and h.current_task is None


def test_path_finish_returns_to_manual():
    io, sup, h, sent, _ = make()
    h.handle([P.START_PATH, 1, "p"])
    assert sup.mode == Mode.AUTO and h.current_path == ("p", True)
    io.calls.clear()
    io.path_done(True, "")
    assert sup.mode == Mode.MANUAL and h.current_path is None
    assert ("set_tracker", False) in io.calls
    assert [m for m in sent if m[0] == P.RESULT] == [[P.RESULT, 1, True, ""]]
    h.handle([P.START_PATH, 2, "p"])  # can run again
    assert sup.mode == Mode.AUTO


def test_path_failure_after_start_returns_to_manual():
    io, sup, h, sent, _ = make()
    h.handle([P.START_PATH, 1, "p"])
    io.path_done(False, "lost the way")
    assert sup.mode == Mode.MANUAL and h.current_path is None


def test_path_cancelled_late_done_is_ignored():
    io, sup, h, sent, _ = make()
    h.handle([P.START_PATH, 1, "p"])
    done = io.path_done
    h.handle([P.CANCEL, 2])
    h.handle([P.START_PATH, 3, "q"])
    done(False, "canceled")  # the old goal ending must not end the new one
    assert sup.mode == Mode.AUTO and h.current_path == ("q", True)


def test_actuation_rejected_while_stopped():
    io, sup, h, sent, _ = make()
    sup.on_control(1.0, ctl(estop=True))
    assert sup.mode == Mode.STOP
    io.calls.clear()
    sent.clear()
    h.handle([P.LIFT_TO, 1, 0, 0.1])
    h.handle([P.LIFT_ZERO, 2, 2])
    h.handle([P.SET_SUCTION, 3, 0, True])
    h.handle([P.SET_TRACKER, 4, True])
    assert [m[:3] for m in sent] == [[P.RESULT, i, False] for i in (1, 2, 3, 4)]
    assert all(m[3] == "stopped: estop" for m in sent)
    assert io.calls == []
    h.handle([P.SET_SUCTION, 5, 0, False])
    h.handle([P.SET_TRACKER, 6, False])
    assert io.calls == [("set_suction", 0, False), ("set_tracker", False)]


def test_shell_opened_respects_datagram_size():
    io, sup, h, sent, _ = make()
    h.shells.replay = lambda tab: b"x" * 2500
    h.handle([P.SHELL_OPEN, 1, "a"])
    assert sent[0][0] == P.SHELL_OPENED and len(sent[0][2]) == P.MAX_SHELL_CHUNK
    assert [m[0] for m in sent] == [P.SHELL_OPENED, P.SHELL_OUTPUT, P.SHELL_OUTPUT, P.RESULT]
    assert b"".join(m[2] for m in sent[:3]) == b"x" * 2500


def test_complete_runs_off_the_calling_thread():
    import threading
    import time
    io, sup, h, sent, _ = make()
    gate = threading.Event()
    h.complete = lambda line, cursor, cwd: (gate.wait(2.0), (0, ["a"]))[1]
    h._spawn = __import__("oumuamua_controller_bridge.handlers", fromlist=["x"])._thread_spawn
    t0 = time.monotonic()
    h.handle([P.SHELL_COMPLETE, 1, "a", "l", 1])
    assert time.monotonic() - t0 < 0.1 and sent == []
    gate.set()
    end = time.monotonic() + 2
    while time.monotonic() < end and len(sent) < 2:
        time.sleep(0.01)
    assert [m[0] for m in sent] == [P.SHELL_COMPLETION, P.RESULT]
