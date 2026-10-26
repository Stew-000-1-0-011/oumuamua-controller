import math

import pytest
from pathlib import Path

from oumuamua_controller_bridge.config import DEFAULTS, build_config, parse_simple_yaml
from oumuamua_controller_bridge.fake_robot import FakeRosIo

BRIDGE_YAML = Path(__file__).resolve().parent.parent / "config" / "bridge.yaml"


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def make():
    clk = Clock()
    return FakeRosIo(clock=clk), clk


def run(io, clk, seconds, dt=0.05):
    for _ in range(int(round(seconds / dt))):
        clk.t += dt
        io.snapshot()


def test_fake_drives_with_cmd_vel():
    io, clk = make()
    io.set_pose(0.0, 0.0, math.pi / 2)
    io.snapshot()
    io.publish_cmd_vel(1.0, 0.0, 0.0)  # body +x = field +y
    run(io, clk, 1.0)
    x, y, yaw = io.snapshot().pose
    assert abs(x) < 1e-6 and abs(y - 1.0) < 0.06
    assert abs(yaw - math.pi / 2) < 1e-6
    assert io.snapshot().vel == (1.0, 0.0, 0.0)


def test_fake_scan_hits_walls():
    io, _ = make()
    io.set_pose(0.0, 0.0, 0.0)
    scan = io.snapshot().scan
    assert len(scan) == 360
    east = min(scan, key=lambda p: abs(p[1]) + abs(p[0] - 2.786))
    assert abs(east[0] - 2.786) < 0.01 and abs(east[1]) < 0.01


def test_fake_task_completes():
    io, clk = make()
    steps, results = [], []
    io.run_task(0, 0, 2, 0, 0, steps.append, lambda ok, m: results.append((ok, m)))
    run(io, clk, 0.5)
    assert results == [] and steps
    run(io, clk, 5.0)
    assert results == [(True, "")]
    assert len(steps) >= 2
    assert "yes" in io.snapshot().task_text.splitlines()[0]  # holding after PICK


def test_fake_task_cancel_reports_done():
    io, clk = make()
    results = []
    io.run_task(1, 0, 0, 0, 0, lambda s: None, lambda ok, m: results.append(ok))
    run(io, clk, 0.5)
    io.cancel_task()
    assert results == [False]


def test_fake_lift_moves_and_cancels():
    io, clk = make()
    results = []
    io.lift_to(0, 0.1, lambda ok, m: results.append(("a", ok)))
    run(io, clk, 1.0)
    g = io.snapshot().grippers[0]
    assert abs(g[0] - 0.05) < 0.005 and g[2] is True  # 0.05 m/s, still moving
    run(io, clk, 1.5)
    assert results == [("a", True)]
    assert io.snapshot().grippers[0][3] is True  # arrived
    io.lift_to(0, 0.0, lambda ok, m: results.append(("b", ok)))
    run(io, clk, 0.5)
    io.lift_cancel(0)
    assert results[-1] == ("b", False)
    h = io.snapshot().grippers[0][0]
    run(io, clk, 1.0)
    assert io.snapshot().grippers[0][0] == h  # stopped


def test_fake_suction_and_tracker_and_zero():
    io, clk = make()
    out = []
    io.set_suction(1, True, lambda ok, m: out.append(ok))
    io.set_tracker(True, lambda ok, m: out.append(ok))
    snap = io.snapshot()
    assert out == [True, True] and snap.grippers[1][4] is True and snap.tracker_enabled
    io.lift_to(0, 0.05, lambda ok, m: None)
    run(io, clk, 2.0)
    io.lift_zero(0, lambda ok, m: out.append(ok))
    assert io.snapshot().grippers[0][0] == 0.0


def test_default_config_matches_bridge_yaml():
    params = parse_simple_yaml(BRIDGE_YAML.read_text(encoding="utf-8"))
    assert build_config(params) == DEFAULTS


def test_fake_path_run_finishes_after_a_few_seconds():
    io, clk = make()
    ev = []
    io.start_path("slalom_forward", lambda ok, m: ev.append("started"),
                  lambda ok, m: ev.append(("done", ok)))
    assert ev == ["started"] and io.snapshot().path
    run(io, clk, 2.0)
    assert ev == ["started"]
    run(io, clk, 1.5)
    assert ev == ["started", ("done", True)] and not io.snapshot().path


def test_fake_stop_path_cancels_run():
    io, clk = make()
    ev = []
    io.start_path("p", lambda ok, m: None, lambda ok, m: ev.append(("done", ok)))
    io.stop_path(lambda ok, m: None)
    assert ev == [("done", False)]


def test_fake_motors_follow_the_command_and_suction():
    io, clk = make()
    motors = io.snapshot().wheels
    assert len(motors) == 5
    assert all(m[1] for m in motors)
    assert [m[4] for m in motors[:3]] == [2, 2, 2]  # velocity mode even at rest
    io.publish_cmd_vel(1.0, 0.0, 0.0)
    wheels = io.snapshot().wheels[:3]
    assert any(abs(w[3]) > 1.0 for w in wheels)
    for w in wheels:  # current is proportional to speed
        assert w[2] == pytest.approx(0.06 * w[3])
    io.publish_cmd_vel(2.0, 0.0, 0.0)
    assert abs(io.snapshot().wheels[1][3]) == pytest.approx(2 * abs(wheels[1][3]))
    io.set_suction(1, True, lambda ok, m: None)
    sm = io.snapshot().wheels[3:]
    assert sm[0][2] == 0.0 and sm[0][4] == 0  # left off: disabled
    assert sm[1][2] == pytest.approx(2.5) and sm[1][4] == 2  # right on: draws current
