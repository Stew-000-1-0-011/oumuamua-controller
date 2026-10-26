"""The ROS-free half of the motor snapshot: Status fields -> snapshot entry -> STATE row."""
from types import SimpleNamespace

import pytest

from oumuamua_controller_bridge import protocol as P
from oumuamua_controller_bridge.rosio import motor_entry, motor_fields


def status(**kw):
    base = dict(mode=2, saturated_upper=False, saturated_lower=False, vdc_low=False,
                current=1.5, velocity=-20.0, position=0.0, vdc=23.5, temperature_raw=0)
    base.update(kw)
    return SimpleNamespace(**base)


def test_fields_copy_current_velocity_mode_and_pack_the_flags():
    assert motor_fields(status()) == (23.5, 1.5, -20.0, 2, 0)
    f = motor_fields(status(saturated_upper=True, vdc_low=True))
    assert f[4] == P.SATURATED_UPPER | P.VDC_LOW
    assert motor_fields(status(saturated_lower=True))[4] == P.SATURATED_LOWER


def test_entry_is_ok_only_when_fresh_and_not_in_fault():
    f = motor_fields(status())
    assert motor_entry(f, True) == (23.5, True, 1.5, -20.0, 2, 0)
    assert motor_entry(f, False)[1] is False  # stale: last values, flagged
    fault = motor_fields(status(mode=P.MODE_FAULT))
    assert motor_entry(fault, True)[1] is False


def test_entry_before_any_message_is_not_ok_and_has_no_detail():
    assert motor_entry(None, False) == (0.0, False, 0.0, 0.0, P.MODE_UNKNOWN, 0)


def test_entry_becomes_a_state_row():
    row = P.motor_row(3, motor_entry(motor_fields(status(saturated_lower=True)), True))
    assert row == [23.5, True, 1.5, -20.0, 2, 2, "吸引 左"]
    assert P.encode(row)
    assert pytest.approx(row[0]) == 23.5
