#!/usr/bin/env python3
"""Deterministic golden samples. Usage: make_golden.py <output dir>"""
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from oumuamua_controller_bridge import protocol as p  # noqa: E402

S = 0x12345678  # session
GRIPPER = [0.5, 0.25, True, False, True, False]

STATE_FULL = [
    1, "", [1.5, -0.25, -1.25], [0.5, -0.5, 0.125],
    [[12.5, True, 1.25, -8.5, 2, 0, "車輪0"], [-3.75, False, 0.0, 0.0, 4, 3, "車輪1"],
     [0.0, True, -0.5, 2.0, 1, 4, "車輪2"], [24.0, True, 0.75, 6.25, 2, 0, "吸引 左"],
     [23.5, True, 0.0, 0.0, 0, 0, "吸引 右"]],
    [GRIPPER, [1.25, 1.25, False, True, False, True]],
    [3, "grab", [0, -1, 2, 1]],
    ["lane_a", True],
    True,
]
STATE_EMPTY = [2, "link lost", None, None, [], [None, None], None, None, False]

POINTS = struct.pack("<8h", 1500, -250, 0, 32767, -32767, 1, 3, -4)
SHELL_DATA = b"ls -la\n\x03\xff\x00"

DATAGRAMS = {
    "hello": [p.HELLO, S, "pixel", False, None],
    "welcome": [p.WELCOME, S, "oumuamua", True, ""],
    "control": [p.CONTROL, S, 7, 123456, 0.5, -1.25, 0.0, True, -1, 1, False],
    "state_full": [p.STATE, S, 42, 7, 123456, STATE_FULL],
    "state_empty": [p.STATE, S, 43, 0, 0, STATE_EMPTY],
    "scan": [p.SCAN, S, 9, 0, 1, POINTS],
    "ack": [p.ACK, S, 5],
    "beacon": [p.BEACON, "oumuamua", p.PORT],
    "bye": [p.BYE, S, "taken over"],
}

MSGS = {
    "release_stop": [p.RELEASE_STOP, 1],
    "set_suction": [p.SET_SUCTION, 2, 2, True],
    "lift_to": [p.LIFT_TO, 3, 0, 0.5],
    "lift_zero": [p.LIFT_ZERO, 4, 1],
    "run_task": [p.RUN_TASK, 5, 3, -1, 2, 0, 1],
    "cancel": [p.CANCEL, 6],
    "start_path": [p.START_PATH, 7, "lane_a"],
    "set_tracker": [p.SET_TRACKER, 8, False],
    "set_pose": [p.SET_POSE, 9, -1.25, 0.5, -0.125],
    "task_reset": [p.TASK_RESET, 10],
    "log_level": [p.LOG_LEVEL, 11, 30],
    "shell_open": [p.SHELL_OPEN, 12, "main"],
    "shell_input": [p.SHELL_INPUT, 13, "main", SHELL_DATA],
    "shell_complete": [p.SHELL_COMPLETE, 14, "main", "ros2 top", 8],
    "shell_close": [p.SHELL_CLOSE, 15, "main"],
    "shell_rename": [p.SHELL_RENAME, 16, "main", "main2"],
    "result": [p.RESULT, 5, False, "busy"],
    "config": [
        p.CONFIG, "oumuamua", ["lane_a", "lane_b"],
        [["low", 0.25], ["high", 0.75]], 1.5,
        [[0.5, 1.0], [1.0, 1.5], [2.0, 2.5]], 4, 3, 2,
    ],
    "field": [
        p.FIELD, [-3.0, -2.0, 3.0, 2.0], [[-1.0, -0.5, -0.5, 0.5], [0.0, 1.0, 0.25, 1.5]],
        [1.0, -0.5, 2.0, 0.5], [-2.5, 0.0, -1.25], 0.375,
    ],
    "path_shape": [p.PATH_SHAPE, "lane_a", POINTS],
    "task_memory": [p.TASK_MEMORY, [True, False], [0, 2, 3], [[1, 2], [0, 0]]],
    "task_done": [p.TASK_DONE, 3, True, "ok"],
    "log": [p.LOG, 1761100000123, 20, "bridge", "started"],
    "shell_opened": [p.SHELL_OPENED, "main", b"$ \x1b[0m"],
    "shell_output": [p.SHELL_OUTPUT, "main", SHELL_DATA],
    "shell_completion": [p.SHELL_COMPLETION, 14, "main", 4, ["top", "topic"]],
    "shell_closed": [p.SHELL_CLOSED, "main", "exited"],
}

SAMPLES = dict(DATAGRAMS)
for _i, (_name, _msg) in enumerate(MSGS.items(), start=1):
    SAMPLES["rel_" + _name] = [p.REL, S, _i, _msg]


def main() -> None:
    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    for name, value in SAMPLES.items():
        (out / f"{name}.msgpack").write_bytes(p.encode(value))


if __name__ == "__main__":
    main()
