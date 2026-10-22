"""Parser for /task_commander/state text (see task_commander_node.cpp publish_state)."""
import re

_HOLD = re.compile(r"=(yes|no)\b")
_CELL = re.compile(r"\(\s*(\d+)\s*,\s*(\d+)\s*\)")


def parse(text: str):
    """Return (holding [gripper0, gripper1], picked block ids, placed (column, row) cells)."""
    holding = [False, False]
    picked = []
    placed = []
    for line in text.splitlines():
        if line.startswith("holding:"):
            vals = [m == "yes" for m in _HOLD.findall(line)]
            for i, v in enumerate(vals[:2]):
                holding[i] = v
        elif line.startswith("picked blocks:"):
            picked = [int(t) for t in line.split(":", 1)[1].split() if t.isdigit()]
        elif line.startswith("placed cells"):
            body = line.split(":", 1)[1]
            placed = [(int(c), int(r)) for c, r in _CELL.findall(body)]
    return holding, picked, placed
