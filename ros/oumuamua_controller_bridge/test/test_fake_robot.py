import math

import pytest
from pathlib import Path

from oumuamua_controller_bridge.config import (
    DEFAULTS, build_config, from_ros_params, parse_simple_yaml, robot_shape_from_text)
from oumuamua_controller_bridge.fake_robot import FakeRosIo

# The bridge's parameter file is in oumuamua_bringup (the bridge has no defaults). When this repository is
# checked out as oumuamua's src/oumuamua-controller, oumuamua_bringup is next to it.
BRINGUP_CONFIG = Path(__file__).resolve().parents[4] / "oumuamua_bringup" / "config"
BRIDGE_YAML = BRINGUP_CONFIG / "controller_bridge.yaml"
FIELD_YAML = BRINGUP_CONFIG / "field.yaml"
TASK_COMMANDER_YAML = BRINGUP_CONFIG / "task_commander.yaml"


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
    east = min(scan, key=lambda p: abs(p[1]) + abs(p[0] - 2.785))
    assert abs(east[0] - 2.785) < 0.01 and abs(east[1]) < 0.01


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
    if not BRIDGE_YAML.exists():
        pytest.skip(f"{BRIDGE_YAML} not found (not inside the oumuamua workspace)")
    params = parse_simple_yaml(BRIDGE_YAML.read_text(encoding="utf-8"))
    # field.yaml nests its keys under field:, which parse_simple_yaml flattens away
    params |= {"field." + k: v for k, v in parse_simple_yaml(FIELD_YAML.read_text(encoding="utf-8")).items()}
    # footprint and grippers.suction_offsets come from task_commander.yaml (bridge.launch.py reads them there);
    # a copy in controller_bridge.yaml, if any, wins
    if TASK_COMMANDER_YAML.exists():
        params = robot_shape_from_text(TASK_COMMANDER_YAML.read_text(encoding="utf-8")) | params
    assert build_config(from_ros_params(params)) == DEFAULTS


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


def _hold(io):
    return [w == "yes" for w in (t.split("=")[1] for t in io.snapshot().task_text.splitlines()[0].split()[1:])]


def _picked(io):
    return [int(t) for t in io.snapshot().task_text.splitlines()[1].split(":", 1)[1].split()]


def _run_task(io, clk, kind, gripper=0, block=0, column=0, row=0):
    steps, results = [], []
    io.run_task(kind, gripper, block, column, row, steps.append, lambda ok, m: results.append((ok, m)))
    run(io, clk, 5.0)
    return steps, results


def test_to_shelf_cell_only_moves_and_remembers_nothing():
    io, clk = make()
    assert _run_task(io, clk, 0, gripper=1, block=2)[1] == [(True, "")]
    steps, results = _run_task(io, clk, 5, gripper=1, column=1, row=2)
    assert steps == ["マスへ移動"] and results == [(True, "")]
    text = io.snapshot().task_text
    assert _hold(io) == [False, True], "still holding: the operator releases the suction"
    assert "placed" not in text and _picked(io) == [2]


def test_to_shelf_front_has_no_effect_on_what_is_held():
    io, clk = make()
    steps, results = _run_task(io, clk, 7)
    assert steps == ["棚の前へ移動"] and results == [(True, "")]
    assert _hold(io) == [False, False] and _picked(io) == []


def test_pick_pair_holds_both_grippers_and_marks_both_blocks():
    io, clk = make()
    steps, results = _run_task(io, clk, 8, block=4)
    assert results == [(True, "")] and len(steps) == 3
    assert _hold(io) == [True, True] and _picked(io) == [4, 3]
    _run_task(io, clk, 8, block=2)
    assert _picked(io) == [4, 3, 2, 1]


def test_pick_pair_rejects_blocks_without_a_pair_pose():
    io, clk = make()
    results = []
    io.run_task(8, 0, 3, 0, 0, lambda s: None, lambda ok, m: results.append((ok, m)))
    assert results and results[0][0] is False
    assert _hold(io) == [False, False]


def test_releasing_suction_drops_the_held_state():
    io, clk = make()
    _run_task(io, clk, 8, block=4)
    io.set_suction(0, False, lambda ok, m: None)
    assert _hold(io) == [False, True]
    io.set_suction(1, True, lambda ok, m: None)
    assert _hold(io) == [False, True], "switching on does not drop it"
    io.set_suction(1, False, lambda ok, m: None)
    assert _hold(io) == [False, False] and _picked(io) == [4, 3], "picked blocks are kept"


def test_reset_forgets_picked_and_held():
    io, clk = make()
    _run_task(io, clk, 8, block=4)
    io.task_reset(lambda ok, m: None)
    assert _hold(io) == [False, False] and _picked(io) == []


def _ros_params(**extra):
    base = {"robot_name": "x", "lift_max": 0.8, "robot_radius": 0.3, "completion_setup_files": [],
            "lift_preset_names": ["a"], "lift_preset_heights": [0.0], "speed_levels": [0.3, 1.0],
            "field.bounds": [0, 0, 1, 1], "field.walls": [], "field.shelf": [0, 0, 1, 1],
            "field.start_pose": [0, 0, 0], "field.notes_x": [0], "field.shelf_columns_x": [0],
            "field.shelf_rows": 1}
    return base | extra


def test_robot_shape_params_are_optional_and_follow_the_ros_names():
    cfg = build_config(from_ros_params(_ros_params()))
    assert cfg["footprint"] == [] and cfg["suction_offsets"] == []  # older robot files: the map draws a circle
    cfg = build_config(from_ros_params(_ros_params(**{
        "footprint": [0.1, 0.2, 0.3, 0.4], "grippers.suction_offsets": [0.269, 0.12, 0.269, -0.12]})))
    assert cfg["footprint"] == [0.1, 0.2, 0.3, 0.4] and cfg["suction_offsets"] == [0.269, 0.12, 0.269, -0.12]


def test_robot_shape_is_read_out_of_task_commander_yaml_text():
    text = """task_commander:
  ros__parameters:
    odom_topic: /state_estimator/odom
    footprint: [
      0.1, -0.2,   # a comment
      0.3, 0.4,
    ]
    grippers:
      suction_offsets: [0.269, 0.120, 0.269, -0.120]
"""
    assert robot_shape_from_text(text) == {"footprint": [0.1, -0.2, 0.3, 0.4],
                                           "grippers.suction_offsets": [0.269, 0.12, 0.269, -0.12]}
    assert robot_shape_from_text("task_commander:\n  ros__parameters:\n    x: 1\n") == {}
