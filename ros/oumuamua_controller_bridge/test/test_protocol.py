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
