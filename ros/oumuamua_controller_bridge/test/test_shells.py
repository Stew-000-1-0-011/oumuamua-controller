import os
import time
from pathlib import Path

import pytest

from oumuamua_controller_bridge.shells import ShellManager


class Rec:
    def __init__(self):
        self.out = {}
        self.closed = []

    def on_output(self, tab, data):
        self.out[tab] = self.out.get(tab, b"") + data

    def on_closed(self, tab, reason):
        self.closed.append((tab, reason))


@pytest.fixture
def env(tmp_path):
    home = tmp_path / "home"
    home.mkdir()
    rc = tmp_path / "inputrc"
    rc.write_text("set enable-bracketed-paste off\n")  # keep escape codes out of the output
    return {"HOME": str(home), "PATH": os.environ["PATH"], "PS1": "PROMPT$ ", "INPUTRC": str(rc)}


@pytest.fixture
def rig(tmp_path, env):
    rec = Rec()
    sm = ShellManager(tmp_path / "log", rec.on_output, rec.on_closed, env=env)
    yield sm, rec
    sm.close_all()


def pump(sm, rec, tab, cond, limit=2.5):
    end = time.monotonic() + limit
    while time.monotonic() < end:
        sm.poll(0.05)
        if cond(rec.out.get(tab, b"")):
            return True
    return False


def test_open_echo_and_output(rig):
    sm, rec = rig
    assert sm.open("t1") == (True, "")
    assert sm.write("t1", b"echo hi\n")[0]
    assert pump(sm, rec, "t1", lambda o: b"\r\nhi\r\n" in o)
    assert sm.tabs() == ["t1"]


def test_ctrl_c_interrupts(rig):
    sm, rec = rig
    sm.open("t1")
    assert pump(sm, rec, "t1", lambda o: b"PROMPT$ " in o)
    rec.out["t1"] = b""
    sm.write("t1", b"sleep 30\n")
    pump(sm, rec, "t1", lambda o: False, 0.3)
    rec.out["t1"] = b""
    t0 = time.monotonic()
    sm.write("t1", b"\x03")
    assert pump(sm, rec, "t1", lambda o: b"PROMPT$ " in o, 1.0)
    assert time.monotonic() - t0 < 1.0


def test_close_kills_bash(rig):
    sm, rec = rig
    sm.open("t1")
    pid = sm._tabs["t1"].pid
    assert sm.close("t1")[0]
    with pytest.raises(OSError):
        os.kill(pid, 0)
    assert sm.tabs() == []
    assert sm.close("t1")[0] is False


def test_exit_reports_closed(rig):
    sm, rec = rig
    sm.open("t1")
    sm.write("t1", b"exit\n")
    end = time.monotonic() + 2.5
    while time.monotonic() < end and not rec.closed:
        sm.poll(0.05)
    assert rec.closed == [("t1", "exited")]
    assert sm.tabs() == []


def test_duplicate_and_bad_names_rejected(rig):
    sm, _ = rig
    assert sm.open("a-b_1")[0]
    ok, reason = sm.open("a-b_1")
    assert not ok and reason
    for bad in ["", "a b", "x" * 33, "../x", "日本"]:
        ok, reason = sm.open(bad)
        assert not ok and reason
    assert sm.write("nope", b"x")[0] is False
    assert sm.rename("a-b_1", "bad name")[0] is False


def test_replay_keeps_last_lines(rig):
    sm, rec = rig
    sm.open("t1")
    sm.write("t1", b"seq 1 3000\n")
    assert pump(sm, rec, "t1", lambda o: b"3000\r\n" in o)
    lines = sm.replay("t1").split(b"\n")
    assert len(lines) <= 2001
    assert b"3000" in sm.replay("t1")
    assert b"\r\n1\r\n" not in b"\r\n" + sm.replay("t1")
    assert len(sm.replay("t1")) <= 256 * 1024


def test_log_file_written(rig, tmp_path):
    sm, rec = rig
    sm.open("t1")
    sm.write("t1", b"echo hi\n")
    assert pump(sm, rec, "t1", lambda o: b"\r\nhi\r\n" in o)
    sm.write("t1", b"\x03")
    sm.close_all()
    files = list((tmp_path / "log" / "t1").glob("*.log"))
    assert len(files) == 1
    import re
    assert re.fullmatch(r"\d{8}-\d{6}\.log", files[0].name)
    lines = files[0].read_text().splitlines()
    assert any(re.fullmatch(r"\d\d:\d\d:\d\d\.\d{3} > echo hi", l) for l in lines)
    assert any(re.fullmatch(r"\d\d:\d\d:\d\d\.\d{3}   hi", l) for l in lines)
    assert any(l.endswith(" > ^C") for l in lines)


def test_rename_moves_logging(rig, tmp_path):
    sm, rec = rig
    sm.open("old")
    sm.write("old", b"echo one\n")
    assert pump(sm, rec, "old", lambda o: b"one\r\n" in o)
    assert sm.rename("old", "new") == (True, "")
    assert sm.tabs() == ["new"]
    sm.write("new", b"echo two\n")
    end = time.monotonic() + 2
    while time.monotonic() < end and b"two\r\n" not in rec.out.get("new", b""):
        sm.poll(0.05)
    sm.close_all()
    logs = list((tmp_path / "log" / "new").glob("*.log"))
    assert len(logs) == 1
    assert "echo two" in logs[0].read_text()
    assert not (tmp_path / "log" / "old").exists() or not list((tmp_path / "log" / "old").glob("*.log"))
    assert sm.rename("new", "new")[0] is False


def test_cwd_and_poll_timeout(rig, tmp_path):
    sm, rec = rig
    assert sm.cwd("missing") == ""
    sm.open("t1")
    assert Path(sm.cwd("t1")).is_dir()
    pump(sm, rec, "t1", lambda o: b"PROMPT$ " in o)
    t0 = time.monotonic()
    sm.poll(0.2)
    assert time.monotonic() - t0 < 0.5


def test_second_tab_does_not_inherit_master_fds(rig):
    sm, _ = rig
    sm.open("a")
    sm.open("b")
    pid = sm._tabs["b"].pid
    deadline = time.monotonic() + 2
    while True:
        fds = {int(x) for x in os.listdir(f"/proc/{pid}/fd")}
        if fds <= {0, 1, 2, 255} or time.monotonic() > deadline:
            break
        time.sleep(0.02)
    assert fds <= {0, 1, 2, 255}, fds


def test_open_returns_false_when_log_unwritable(tmp_path, env):
    blocker = tmp_path / "blocked"
    blocker.write_text("x")
    sm = ShellManager(blocker, lambda *a: None, lambda *a: None, env=env)
    ok, reason = sm.open("t")
    assert not ok and reason
    assert sm.tabs() == []
