"""Harite (lift without suction): pure helpers and the supervisor's handling of CONTROL.jog_harite."""
import math

import pytest

from fake_rosio import FakeRosIo
from oumuamua_controller_bridge import protocol as P
from oumuamua_controller_bridge.config import DEFAULTS, build_config, from_ros_params
from oumuamua_controller_bridge.harite import (
    DEFAULT_SPEED, MAX_SPEED, RETRY_S, clamp_speed, command_value, jog_sign, target_velocity)
from oumuamua_controller_bridge.supervisor import Mode, Supervisor


def ctl(**kw):
    c = dict(session=1, seq=1, t_ms=0, vx=0.0, vy=0.0, wz=0.0, field_frame=False,
             jog_left=0, jog_right=0, estop=False, jog_harite=0)
    c.update(kw)
    return c


def running(speed=30.0, reversed_=False, auto_done=True):
    """A supervisor in MANUAL (released) with harite; the calls so far are cleared."""
    io = FakeRosIo(auto_done)
    s = Supervisor(io, lift_max=0.5, harite_speed=speed, harite_reversed=reversed_)
    s.on_control(0.0, ctl())
    s.tick(0.0, None)  # stopped: harite is told to disable
    assert s.release() == (True, "")
    io.calls.clear()
    return io, s


def drive(io, s, jog, t, **kw):
    """One CONTROL and one tick at time t; returns what harite was sent during them."""
    n = len(io.calls)
    s.on_control(t, ctl(jog_harite=jog, **kw))
    s.tick(t, None)
    return [c for c in io.calls[n:] if c[0] == "harite"]


# -- pure helpers ------------------------------------------------------------------------------------
def test_clamp_speed():
    assert clamp_speed(30) == 30.0 and isinstance(clamp_speed(30), float)
    assert clamp_speed(250.0) == MAX_SPEED == 100.0
    assert clamp_speed(-5.0) == 0.0
    for bad in (math.nan, math.inf, "fast", None):
        assert clamp_speed(bad) == DEFAULT_SPEED


def test_jog_sign_reads_anything_an_old_or_odd_peer_sends():
    assert [jog_sign(v) for v in (1, -1, 0, 7, -3, 0.5, -0.5)] == [1, -1, 0, 1, -1, 1, -1]
    for bad in (None, True, False, "1", math.nan, [1], b"\x01"):
        assert jog_sign(bad) == 0


def test_target_velocity_sign_speed_and_reversal():
    assert target_velocity(1, 30.0, False) == 30.0
    assert target_velocity(-1, 30.0, False) == -30.0
    assert target_velocity(1, 30.0, True) == -30.0
    assert target_velocity(-1, 30.0, True) == 30.0
    assert target_velocity(1, 500.0, False) == 100.0
    v = target_velocity(0, 30.0, True)
    assert v == 0.0 and math.copysign(1.0, v) == 1.0, "released is +0.0 even when reversed"


def test_command_value_for_state():
    assert command_value(None) is None and command_value(math.nan) is None and command_value(math.inf) is None
    assert command_value(12) == 12.0 and command_value(-0.5) == -0.5


def test_control_jog_harite_from_the_wire():
    base = [P.CONTROL, 1, 1, 0, 0.0, 0.0, 0.0, False, 0, 0, False]
    assert P.control_jog_harite(base) == 0, "an older phone sends 11 elements"
    assert P.control_jog_harite(base + [1]) == 1
    assert P.control_jog_harite(base + [-1]) == -1
    assert P.control_jog_harite(base + [0]) == 0
    assert P.control_jog_harite(base + [None]) == 0
    assert P.control_jog_harite(base + [True]) == 0
    assert P.control_jog_harite(base + [9]) == 1 and P.control_jog_harite(base + [-9]) == -1
    assert P.control_jog_harite(base + [math.nan]) == 0
    assert P.control_jog_harite(base + ["up"]) == 0


# -- config ---------------------------------------------------------------------------------------------
def _ros_params(**extra):
    p = {"robot_name": "r", "lift_preset_names": ["a"], "lift_preset_heights": [0.1], "lift_max": 0.8,
         "speed_levels": [0.3, 1.0, 0.8, 1.8, 1.5, 2.5], "robot_radius": 0.3, "completion_setup_files": [],
         "field.bounds": [0, 0, 1, 1], "field.walls": [0, 0, 1, 1], "field.shelf": [0, 0, 1, 1],
         "field.notes_x": [0.1] * 5, "field.shelf_columns_x": [0.1] * 3, "field.shelf_rows": 3,
         "field.start_pose": [0, 0, 0]}
    return p | extra


def test_harite_parameters_are_optional_with_defaults():
    cfg = build_config(from_ros_params(_ros_params()))
    assert (cfg["harite_enabled"], cfg["harite_speed"], cfg["harite_reversed"]) == (True, 30.0, False)
    assert (DEFAULTS["harite_enabled"], DEFAULTS["harite_speed"], DEFAULTS["harite_reversed"]) == (True, 30.0, False)


def test_harite_parameters_are_read_and_the_speed_clamped():
    cfg = build_config(from_ros_params(_ros_params(harite_speed=12, harite_reversed=True, harite_enabled=False)))
    assert (cfg["harite_enabled"], cfg["harite_speed"], cfg["harite_reversed"]) == (False, 12.0, True)
    assert build_config(from_ros_params(_ros_params(harite_speed=1e3)))["harite_speed"] == 100.0
    assert build_config(from_ros_params(_ros_params(harite_speed=-4.0)))["harite_speed"] == 0.0


# -- supervisor: manual -------------------------------------------------------------------------------------
def test_publishes_while_held_and_zero_on_release():
    io, s = running()
    assert drive(io, s, 1, 1.0) == [("harite", 30.0)]
    assert drive(io, s, 1, 1.02) == [("harite", 30.0)]
    assert drive(io, s, 0, 1.04) == [("harite", 0.0)], "released: 0, not silence"
    assert drive(io, s, 0, 1.06) == [("harite", 0.0)], "and 0 keeps coming while manual"
    assert drive(io, s, -1, 1.08) == [("harite", -30.0)]


def test_every_tick_publishes_even_between_controls():
    io, s = running()
    s.on_control(1.0, ctl(jog_harite=1))
    for t in (1.0, 1.02, 1.04, 1.06):
        s.tick(t, None)
    assert io.harite_values() == [30.0] * 4


def test_reversed_flips_the_sign():
    io, s = running(reversed_=True)
    assert drive(io, s, 1, 1.0) == [("harite", -30.0)]
    assert drive(io, s, -1, 1.02) == [("harite", 30.0)]


def test_speed_is_clamped_to_zero_and_one_hundred():
    io, s = running(speed=500.0)
    assert drive(io, s, 1, 1.0) == [("harite", 100.0)]
    io, s = running(speed=-3.0)
    assert drive(io, s, 1, 1.0) == [("harite", 0.0)]


def test_an_older_phone_without_jog_harite_never_moves_it():
    io, s = running()
    c = ctl()
    del c["jog_harite"]
    s.on_control(1.0, c)
    s.tick(1.0, None)
    assert io.harite_values() == [0.0]


# -- supervisor: stop ----------------------------------------------------------------------------------
def test_estop_publishes_zero_then_disables():
    io, s = running()
    drive(io, s, 1, 1.0)
    io.calls.clear()
    s.on_control(1.02, ctl(jog_harite=1, estop=True))
    assert s.mode == Mode.STOP
    hs = [c for c in io.calls if c[0] in ("harite", "harite_enable")]
    assert hs[:2] == [("harite", 0.0), ("harite_enable", False)], "0 first, then disable"
    # The same tick that follows must not publish the held direction.
    s.tick(1.02, None)
    assert all(v == 0.0 for v in io.harite_values())


def test_link_loss_publishes_zero_then_disables():
    io, s = running()
    drive(io, s, 1, 1.0)
    io.calls.clear()
    s.tick(1.31, None)  # 0.3 s without CONTROL
    assert s.mode == Mode.STOP and s.stop_reason == "link lost"
    hs = [c for c in io.calls if c[0] in ("harite", "harite_enable")]
    assert hs[:2] == [("harite", 0.0), ("harite_enable", False)]


def test_link_lost_call_stops_it_too():
    io, s = running()
    s.link_lost()
    hs = [c for c in io.calls if c[0] in ("harite", "harite_enable")]
    assert hs == [("harite", 0.0), ("harite_enable", False)]


def test_nothing_moves_while_stopped_even_if_a_button_is_still_held():
    io, s = running()
    s.on_control(1.0, ctl(estop=True))
    io.calls.clear()
    for i in range(5):
        s.on_control(1.02 + i * 0.02, ctl(estop=True, jog_harite=1))
        s.tick(1.02 + i * 0.02, None)
    assert all(v == 0.0 for v in io.harite_values())


def test_stop_keeps_sending_zero_until_the_disable_is_confirmed_then_goes_quiet():
    io, s = running(auto_done=False)
    s.on_control(1.0, ctl(estop=True))
    assert [n for n, _ in io.pending if n == "harite_enable"]
    s.tick(1.02, None)
    s.tick(1.04, None)
    assert io.harite_values()[-2:] == [0.0, 0.0], "unconfirmed: keep sending 0"
    done = [d for n, d in io.pending if n == "harite_enable"][-1]
    done(True, "")
    n = len(io.calls)
    s.tick(1.06, None)
    s.tick(1.08, None)
    assert [c for c in io.calls[n:] if c[0] == "harite"] == [], "disabled and confirmed: left alone"


def test_a_failed_disable_is_retried_every_second_not_every_tick():
    io, s = running()
    io.fail["harite_enable"] = "/harite/enable がありません"
    s.on_control(1.0, ctl(estop=True))
    enables = lambda: [c for c in io.calls if c == ("harite_enable", False)]  # noqa: E731
    assert len(enables()) == 1
    for i in range(1, 40):
        s.tick(1.0 + i * 0.02, None)
    assert len(enables()) == 1, "within a second: no new call"
    s.tick(1.0 + RETRY_S + 0.01, None)
    assert len(enables()) == 2
    io.fail.clear()
    s.tick(1.0 + 2 * RETRY_S + 0.02, None)
    assert len(enables()) == 3
    for i in range(1, 100):  # confirmed: no more calls
        s.tick(3.1 + i * 0.02, None)
    assert len(enables()) == 3


def test_starts_stopped_and_disables_harite_until_released():
    io = FakeRosIo()
    s = Supervisor(io, lift_max=0.5, harite_speed=30.0)
    assert io.calls == [], "nothing at construction"
    s.tick(0.0, None)
    assert ("harite_enable", False) in io.calls and io.harite_values() == [0.0]


# -- supervisor: release -----------------------------------------------------------------------------
def test_release_enables_harite():
    io = FakeRosIo()
    s = Supervisor(io, lift_max=0.5, harite_speed=30.0)
    s.on_control(0.0, ctl())
    s.tick(0.0, None)
    io.calls.clear()
    assert s.release() == (True, "")
    assert ("harite_enable", True) in io.calls


def test_a_failed_release_does_not_enable():
    io = FakeRosIo()
    s = Supervisor(io, lift_max=0.5, harite_speed=30.0)
    s.on_control(0.0, ctl(estop=True))
    io.calls.clear()
    assert s.release()[0] is False
    assert ("harite_enable", True) not in io.calls


def test_enable_is_repeated_until_confirmed_so_a_late_node_still_works():
    io = FakeRosIo()
    s = Supervisor(io, lift_max=0.5, harite_speed=30.0)
    s.on_control(0.0, ctl())
    s.tick(0.0, None)
    io.fail["harite_enable"] = "no service"
    s.release()
    ups = lambda: [c for c in io.calls if c == ("harite_enable", True)]  # noqa: E731
    assert len(ups()) == 1
    io.fail.clear()
    s.on_control(0.5, ctl())
    s.tick(0.5, None)
    assert len(ups()) == 1
    s.on_control(1.2, ctl())
    s.tick(1.2, None)
    assert len(ups()) == 2


def test_a_button_held_through_the_stop_waits_to_be_released_and_pressed_again():
    io, s = running()
    drive(io, s, 1, 1.0)
    s.on_control(1.02, ctl(estop=True, jog_harite=1))
    s.on_control(1.04, ctl(jog_harite=1))  # estop cleared on the phone, still holding
    assert s.release() == (True, "")
    assert drive(io, s, 1, 1.06) == [("harite", 0.0)], "still held: ignored"
    assert drive(io, s, 1, 1.08) == [("harite", 0.0)]
    assert drive(io, s, 0, 1.10) == [("harite", 0.0)]
    assert drive(io, s, 1, 1.12) == [("harite", 30.0)], "pressed again: moves"


def test_the_first_control_after_the_release_is_what_drives():
    io, s = running()
    s.on_control(1.0, ctl(estop=True))
    s.on_control(1.02, ctl())
    s.release()
    io.calls.clear()
    s.tick(1.04, None)  # a tick before any new CONTROL
    assert io.harite_values() == [0.0]


def test_a_late_enable_true_reply_after_a_stop_is_not_trusted():
    io = FakeRosIo(auto_done=False)
    s = Supervisor(io, lift_max=0.5, harite_speed=30.0)
    s.on_control(0.0, ctl())
    s.release()  # enable(true) goes out and is not answered
    s.on_control(0.1, ctl(estop=True))  # stop again: enable(false) goes out
    up_done, down_done = [d for n, d in io.pending if n == "harite_enable"]
    down_done(True, "")
    up_done(True, "")  # the older request lands last: what the node holds is unknown
    n = len(io.calls)
    s.tick(0.1 + RETRY_S + 0.01, None)
    assert ("harite_enable", False) in io.calls[n:], "unknown again: disable is sent once more"


# -- supervisor: auto ------------------------------------------------------------------------------------
def test_auto_publishes_nothing_after_one_zero():
    io, s = running()
    drive(io, s, 1, 1.0)
    io.calls.clear()
    s.enter_auto("task")
    assert io.harite_values() == [0.0]
    io.calls.clear()
    for i in range(10):
        s.on_control(1.02 + i * 0.02, ctl(jog_harite=1))
        s.tick(1.02 + i * 0.02, None)
    assert io.harite_values() == [], "AUTO: not touched; the node's 0.2 s timeout does the rest"
    assert ("harite_enable", False) not in io.calls


def test_a_button_held_through_auto_does_nothing_after_it():
    io, s = running()
    s.enter_auto("task")
    s.on_control(1.0, ctl(jog_harite=1))
    s.leave_auto()
    assert drive(io, s, 1, 1.02) == [("harite", 0.0)]
    assert drive(io, s, 0, 1.04) == [("harite", 0.0)]
    assert drive(io, s, 1, 1.06) == [("harite", 30.0)]


# -- supervisor: robot without harite --------------------------------------------------------------
def test_without_harite_nothing_is_published_or_called():
    io = FakeRosIo()
    s = Supervisor(io, lift_max=0.5)
    s.on_control(0.0, ctl(jog_harite=1))
    s.tick(0.0, None)
    s.release()
    s.on_control(0.1, ctl(jog_harite=1))
    s.tick(0.1, None)
    s.on_control(0.2, ctl(estop=True, jog_harite=1))
    s.enter_auto("x")
    assert not [c for c in io.calls if c[0].startswith("harite")]


def test_a_publisher_that_raises_does_not_break_the_stop():
    io = FakeRosIo()
    s = Supervisor(io, lift_max=0.5, harite_speed=30.0)
    s.on_control(0.0, ctl())
    s.release()

    def boom(v):
        raise RuntimeError("boom")

    io.publish_harite = boom
    s.on_control(0.1, ctl(estop=True))
    assert s.mode == Mode.STOP
    assert ("set_suction", 0, False) in io.calls and ("harite_enable", False) in io.calls
