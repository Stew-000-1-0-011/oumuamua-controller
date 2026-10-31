"""Wire format shared with the phone app (see docs/protocol.md). No ROS imports."""
import struct
from typing import Optional, Sequence

import msgpack

PORT = 47800
BEACON_PORT = 47801

# Datagram kinds
HELLO = 0
WELCOME = 1
CONTROL = 2
STATE = 3
SCAN = 4
REL = 5
ACK = 6
BEACON = 7
BYE = 8

# Msg types: phone -> bridge
RELEASE_STOP = 10
SET_SUCTION = 11
LIFT_TO = 12
LIFT_ZERO = 13
RUN_TASK = 14
CANCEL = 15
START_PATH = 16
SET_TRACKER = 17
SET_POSE = 18
TASK_RESET = 19
LOG_LEVEL = 20
SHELL_OPEN = 30
SHELL_INPUT = 31
SHELL_COMPLETE = 32
SHELL_CLOSE = 33
SHELL_RENAME = 34

# Msg types: bridge -> phone
RESULT = 50
CONFIG = 51
FIELD = 52
PATH_SHAPE = 53
TASK_MEMORY = 54
TASK_DONE = 55
LOG = 56
SHELL_OPENED = 60
SHELL_OUTPUT = 61
SHELL_COMPLETION = 62
SHELL_CLOSED = 63

# Motors in STATE.wheels: the three wheels, then the two suction motors (mini-shirasu boards).
WHEEL_COUNT = 3
SUCTION_COUNT = 2
MOTOR_NAMES = ["車輪0", "車輪1", "車輪2", "吸引 左", "吸引 右"]
# mini_shirasu_ros/msg/Status.mode, and "no detail" for a motor never heard from.
MODE_DISABLED, MODE_CURRENT, MODE_VELOCITY, MODE_POSITION, MODE_FAULT = 0, 1, 2, 3, 4
MODE_UNKNOWN = 255
# Bits of the motor entry's flags.
SATURATED_UPPER = 1
SATURATED_LOWER = 2
VDC_LOW = 4

MAX_POINTS_PER_SCAN = 280
MAX_SHELL_CHUNK = 900


def encode(value: list) -> bytes:
    return msgpack.packb(value, use_bin_type=True, use_single_float=False)


def decode(data: bytes) -> Optional[list]:
    """Return the array, or None if data is broken or not an array led by an int."""
    try:
        value = msgpack.unpackb(data, raw=False)
    except Exception:
        return None
    if not isinstance(value, list) or not value:
        return None
    head = value[0]
    if isinstance(head, bool) or not isinstance(head, int):
        return None
    return value


def control_jog_harite(m) -> int:
    """CONTROL's trailing `jog_harite` (index 11) as -1, 0 or +1. An older phone sends no such element
    (nil, a bool or a non-number also read as 0); a value beyond 1 counts as a press in that direction."""
    if len(m) <= 11:
        return 0
    v = m[11]
    if isinstance(v, bool) or not isinstance(v, (int, float)) or v != v or v == 0:
        return 0
    return 1 if v > 0 else -1


def _to_mm(v: float) -> int:
    return max(-32767, min(32767, int(round(v * 1000.0))))


def pack_points(points: Sequence[tuple]) -> list:
    """Metres -> i16 LE millimetres (x, y), split into chunks of MAX_POINTS_PER_SCAN."""
    if not points:
        return [b""]
    chunks = []
    for i in range(0, len(points), MAX_POINTS_PER_SCAN):
        part = points[i : i + MAX_POINTS_PER_SCAN]
        flat = [_to_mm(c) for p in part for c in p]
        chunks.append(struct.pack(f"<{len(flat)}h", *flat))
    return chunks


def unpack_points(data: bytes) -> list:
    n = len(data) // 4
    flat = struct.unpack(f"<{n * 2}h", data[: n * 4])
    return [(flat[2 * i] / 1000.0, flat[2 * i + 1] / 1000.0) for i in range(n)]


def shell_opened_msgs(tab: str, replay: bytes) -> list:
    """SHELL_OPENED plus SHELL_OUTPUT chunks so that every REL fits one datagram."""
    msgs = [[SHELL_OPENED, tab, replay[:MAX_SHELL_CHUNK]]]
    for i in range(MAX_SHELL_CHUNK, len(replay), MAX_SHELL_CHUNK):
        msgs.append([SHELL_OUTPUT, tab, replay[i:i + MAX_SHELL_CHUNK]])
    return msgs


def motor_row(index: int, m) -> list:
    """One STATE.wheels entry `[vdc, ok, current, velocity, mode, flags, name]` from a snapshot
    motor `(vdc, ok[, current, velocity, mode, flags])`. A short tuple (only voltage and ok) gets
    the "no detail" defaults, mirroring how the phone decodes an older peer."""
    vdc, ok = float(m[0]), bool(m[1])
    current = float(m[2]) if len(m) > 2 else 0.0
    velocity = float(m[3]) if len(m) > 3 else 0.0
    mode = int(m[4]) if len(m) > 4 else MODE_UNKNOWN
    flags = int(m[5]) if len(m) > 5 else 0
    name = MOTOR_NAMES[index] if 0 <= index < len(MOTOR_NAMES) else f"モータ{index}"
    return [vdc, ok, current, velocity, mode, flags, name]
