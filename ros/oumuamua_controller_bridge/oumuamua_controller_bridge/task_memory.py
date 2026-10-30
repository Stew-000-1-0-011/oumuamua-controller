"""Parser for /task_commander/state text (see task_commander_node.cpp publish_state)."""
import re

_HOLD = re.compile(r"=(yes|no)\b")


def parse(text: str):
    """Return (holding [gripper0, gripper1], picked block ids).

    task_commander no longer remembers placed cells (the state has no "placed cells" line), so there is
    nothing to parse for them: the TASK_MEMORY message still carries a `placed` list for wire compatibility
    and the bridge always sends it empty (server.py). An old robot's "placed cells" line is ignored.
    """
    holding = [False, False]
    picked = []
    for line in text.splitlines():
        if line.startswith("holding:"):
            vals = [m == "yes" for m in _HOLD.findall(line)]
            for i, v in enumerate(vals[:2]):
                holding[i] = v
        elif line.startswith("picked blocks:"):
            picked = [int(t) for t in line.split(":", 1)[1].split() if t.isdigit()]
    return holding, picked
