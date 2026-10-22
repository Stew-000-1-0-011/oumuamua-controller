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

    h = Handlers(io, sup, FakeShells(), complete, sent.append, {"lift_max": 0.5})
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


def test_run_task_enters_auto_and_done_returns_to_manual():
    io, sup, h, sent, _ = make(auto_done=False)
    h.handle([P.RUN_TASK, 1, 2, 0, 1, 2, 3])
    assert io.calls == [("run_task", 2, 0, 1, 2, 3)]
    assert sent == [[P.RESULT, 1, True, ""]]
    assert sup.mode == Mode.AUTO
    assert h.current_task == (2, "", (0, 1, 2, 3))
    io.task_step("lift up")
    assert h.current_task == (2, "lift up", (0, 1, 2, 3))
    io.task_done(True, "ok")
    assert sent[-1] == [P.TASK_DONE, 2, True, "ok"]
    assert h.current_task is None and sup.mode == Mode.MANUAL


def test_busy_rejects_second_task():
    io, sup, h, sent, _ = make(auto_done=False)
    h.handle([P.RUN_TASK, 1, 2, 0, 1, 2, 3])
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
    assert [r[2] for r in sent] == [True, True]


def test_malformed_gets_failed_result():
    io, sup, h, sent, _ = make()
    h.handle([P.LIFT_TO, 1])
    assert sent[-1][:3] == [P.RESULT, 1, False]
    h.handle([999, 1])
    h.handle([])
