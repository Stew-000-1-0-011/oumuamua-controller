import math
import pytest
from fake_rosio import FakeRosIo
from oumuamua_controller_bridge.supervisor import Supervisor, Mode


def ctl(**kw):
    c = dict(session=1, seq=1, t_ms=0, vx=0.0, vy=0.0, wz=0.0, field_frame=False,
             jog_left=0, jog_right=0, estop=False)
    c.update(kw)
    return c


def make():
    io = FakeRosIo()
    return io, Supervisor(io, lift_max=0.5)


def running():
    io, s = make()
    s.on_control(0.0, ctl())
    assert s.release() == (True, "")
    io.calls.clear()
    return io, s


STOP_CALLS = [("cancel_task",), ("stop_path",), ("set_tracker", False),
              ("lift_cancel", 0), ("lift_cancel", 1),
              ("set_suction", 0, False), ("set_suction", 1, False)]


def test_starts_stopped_until_release():
    io, s = make()
    assert s.mode == Mode.STOP and s.stop_reason == "link lost"
    ok, msg = s.release()
    assert not ok and "no control" in msg
    s.on_control(0.0, ctl())
    assert s.release() == (True, "")
    assert s.mode == Mode.MANUAL and s.stop_reason == ""


def test_timeout_enters_stop_once():
    io, s = running()
    s.on_control(1.0, ctl())
    s.tick(1.29, None)
    assert s.mode == Mode.MANUAL
    s.tick(1.31, None)
    assert s.mode == Mode.STOP and s.stop_reason == "link lost"
    for t in (1.33, 1.35, 1.37):
        s.tick(t, None)
    assert io.non_vel() == STOP_CALLS


def test_estop_enters_stop_and_blocks_release():
    io, s = running()
    s.on_control(1.0, ctl(estop=True))
    assert s.mode == Mode.STOP and s.stop_reason == "estop"
    s.on_control(1.02, ctl(estop=True))
    assert io.non_vel() == STOP_CALLS
    ok, msg = s.release()
    assert not ok and "estop is still pressed" in msg
    s.on_control(1.04, ctl(estop=False))
    assert s.release() == (True, "")


def test_release_fails_when_control_stale():
    io, s = running()
    s.on_control(1.0, ctl(estop=True))
    s.on_control(1.02, ctl())
    s.tick(2.0, None)
    ok, msg = s.release()
    assert not ok and "no control" in msg


def test_stop_publishes_zero_each_tick():
    io, s = make()
    s.tick(0.0, None)
    s.tick(0.02, None)
    assert io.calls == [("cmd_vel", 0.0, 0.0, 0.0)] * 2


def test_manual_publishes_last_and_clamps():
    io, s = running()
    s.on_control(0.0, ctl(vx=3.0, vy=0.0, wz=-4.0))
    s.tick(0.01, None)
    assert io.calls[-1] == ("cmd_vel", 2.0, 0.0, -2.5)
    s.on_control(0.02, ctl(vx=2.0, vy=2.0))
    s.tick(0.03, None)
    _, vx, vy, wz = io.calls[-1]
    assert math.hypot(vx, vy) == pytest.approx(2.0)
    assert vx == pytest.approx(vy)


def test_field_frame_rotates_by_yaw():
    io, s = running()
    s.on_control(0.0, ctl(vx=1.0, field_frame=True))
    s.tick(0.01, math.pi / 2)
    _, vx, vy, _ = io.calls[-1]
    assert vx == pytest.approx(0.0, abs=1e-9) and vy == pytest.approx(-1.0)
    s.tick(0.02, None)  # no yaw: robot frame as-is
    assert io.calls[-1][1:3] == (1.0, 0.0)


def test_auto_publishes_nothing():
    io, s = running()
    s.enter_auto("task")
    assert s.mode == Mode.AUTO
    s.on_control(0.0, ctl(vx=1.0))
    s.tick(0.01, None)
    assert io.calls == []
    s.leave_auto()
    assert s.mode == Mode.MANUAL


def test_jog_starts_and_cancels_lift():
    io, s = running()
    s.on_control(0.0, ctl(jog_left=1))
    s.on_control(0.02, ctl(jog_left=1))
    s.on_control(0.04, ctl(jog_left=0, jog_right=-1))
    assert io.calls == [("lift_to", 0, 0.5), ("lift_cancel", 0), ("lift_to", 1, 0.0)]


def test_jog_ignored_while_stopped():
    io, s = make()
    s.on_control(0.0, ctl(jog_left=1))
    assert [c for c in io.calls if c[0] == "lift_to"] == []


def test_cancel_returns_to_manual():
    io, s = running()
    s.enter_auto("x")
    s.cancel_to_manual()
    assert s.mode == Mode.MANUAL


def test_nonfinite_velocity_is_zero():
    io, s = running()
    s.on_control(0.1, ctl(vx=float("nan"), vy=float("inf"), wz=float("-inf")))
    s.tick(0.1, 0.0)
    assert io.calls[-1] == ("cmd_vel", 0.0, 0.0, 0.0)


def test_release_does_not_apply_stale_velocity():
    io, s = make()
    s.on_control(0.0, ctl(vx=1.0))
    assert s.release() == (True, "")
    s.tick(0.01, 0.0)
    assert io.calls[-1] == ("cmd_vel", 0.0, 0.0, 0.0)
    s.on_control(0.02, ctl(vx=1.0))
    s.tick(0.03, 0.0)
    assert io.calls[-1] == ("cmd_vel", 1.0, 0.0, 0.0)


def test_enter_stop_runs_every_step_even_if_one_raises():
    io, s = running()

    def boom(*a, **k):
        raise RuntimeError("ros down")
    io.cancel_task = boom
    io.stop_path = boom
    io.set_tracker = boom
    s.on_control(0.1, ctl(estop=True))
    assert s.mode == Mode.STOP
    assert ("set_suction", 0, False) in io.calls and ("set_suction", 1, False) in io.calls
    assert ("lift_cancel", 1) in io.calls
