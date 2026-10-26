import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))

from oumuamua_controller_bridge import protocol  # noqa: E402
from make_golden import SAMPLES  # noqa: E402

GOLDEN = ROOT / "testdata" / "golden"


@pytest.mark.parametrize("name", sorted(SAMPLES))
def test_golden_files_match_samples(name):
    assert (GOLDEN / f"{name}.msgpack").read_bytes() == protocol.encode(SAMPLES[name])


def test_no_stray_golden_files():
    assert {p.stem for p in GOLDEN.glob("*.msgpack")} == set(SAMPLES)


@pytest.mark.parametrize("name", sorted(SAMPLES))
def test_decode_roundtrip(name):
    assert protocol.decode(protocol.encode(SAMPLES[name])) == SAMPLES[name]


def test_decode_rejects_garbage():
    for data in (b"\xc1", protocol.encode([]), protocol.encode(["x"]), protocol.encode({})):
        assert protocol.decode(data) is None


def test_pack_points_splits_and_rounds():
    chunks = protocol.pack_points([(0.0, 0.0)] * 600)
    assert [len(c) // 4 for c in chunks] == [280, 280, 40]
    out = protocol.unpack_points(protocol.pack_points([(0.0015, -40.0)])[0])
    assert out == [(pytest.approx(0.002), pytest.approx(-32.767))]
    assert protocol.pack_points([]) == [b""]
    assert protocol.unpack_points(b"") == []


def test_constants():
    assert (protocol.PORT, protocol.BEACON_PORT) == (47800, 47801)
    assert (protocol.HELLO, protocol.BYE) == (0, 8)
    assert (protocol.RELEASE_STOP, protocol.SHELL_CLOSED) == (10, 63)
    assert (protocol.MAX_POINTS_PER_SCAN, protocol.MAX_SHELL_CHUNK) == (280, 900)


def test_motor_row_defaults_for_a_short_entry_and_names_by_index():
    assert protocol.motor_row(0, (12.5, True)) == [12.5, True, 0.0, 0.0, protocol.MODE_UNKNOWN, 0, "車輪0"]
    assert protocol.motor_row(3, (24, 1, 2.5, 300, 2, 4)) == [24.0, True, 2.5, 300.0, 2, 4, "吸引 左"]
    assert protocol.motor_row(4, (0.0, False))[6] == "吸引 右"
    assert protocol.motor_row(7, (0.0, False))[6] == "モータ7"


def test_state_full_golden_has_five_motors_with_the_extended_layout():
    wheels = SAMPLES["state_full"][5][4]
    assert [w[6] for w in wheels] == protocol.MOTOR_NAMES
    assert all(len(w) == 7 for w in wheels)
    # The first two elements keep the old meaning (vdc, ok): an older phone reads just those.
    assert wheels[0][:2] == [12.5, True]
    assert len(protocol.encode(SAMPLES["state_full"])) <= 1200
