"""Bridge config helpers (no rclpy): flat ROS parameters -> BridgeServer config dict, and a
minimal reader for config/bridge.yaml so the tests need no PyYAML."""
import ast
import os

from .harite import clamp_speed


DEFAULTS = {
    "robot_name": "oumuamua",
    "lift_presets": [["下段", 0.01], ["中段", 0.35], ["上段", 0.69], ["0 点", 0.0]],
    "lift_max": 0.85,
    "speed_levels": [[0.3, 1.0], [0.8, 1.8], [1.5, 2.5]],
    "blocks": 5, "columns": 3, "rows": 3,
    "field_bounds": [-2.785, -1.655, 2.785, 1.655],
    "field_obstacles": [
        -2.8025, -0.02, 1.5625, 0.02,
        -0.905, 0.0225, -0.865, 0.8905,
        -0.905, -0.8905, -0.865, -0.0225,
        -1.735, 1.069, -1.695, 1.669,
        -1.735, -1.669, -1.695, -1.069,
        -1.599, -0.2665, -1.559, -0.02,
        -0.065, 0.8715, -0.025, 1.6715,
        -0.065, -1.6715, -0.025, -0.8715,
        0.765, 0.0225, 0.805, 0.8905,
        0.765, -0.8905, 0.805, -0.0225,
        2.5025, -0.02, 2.8025, 0.02,
        1.5625, -0.3, 2.5025, 0.3,
    ],
    "shelf": [1.5625, -0.30, 2.5025, 0.30],
    "start_pose": [-2.2, 1.3, 3.141592653589793],
    "robot_radius": 0.32,
    # Harite (lift without suction). Optional in controller_bridge.yaml; these are used when it is absent.
    "harite_enabled": True,
    "harite_speed": 30.0,
    "harite_reversed": False,
    # Robot frame (x forward, y left) as in task_commander.yaml: footprint (a flat list of x, y) and
    # grippers.suction_offsets (left gripper first).
    "footprint": [
        0.0834, -0.2255, 0.1582, -0.1824, 0.1589, -0.1818, 0.1593, -0.1809,
        0.1593, 0.1910, 0.1590, 0.1920, 0.1584, 0.1927, 0.0839, 0.2357,
        0.0830, 0.2360, 0.0822, 0.2359, -0.2393, 0.0504, -0.3311, 0.0504,
        -0.3311, -0.0400, -0.2393, -0.0400, 0.0817, -0.2253, 0.0825, -0.2256,
    ],
    "suction_offsets": [0.269, 0.120, 0.269, -0.120],
    "completion_setup_files": ["/opt/ros/$ROS_DISTRO/setup.bash"],
}


# The field's parameters (oumuamua_bringup/config/field.yaml, shared with the other nodes) that the bridge reads.
FIELD_KEYS = ("field.bounds", "field.walls", "field.shelf", "field.notes_x", "field.shelf_columns_x",
              "field.shelf_rows", "field.start_pose")


# The robot's shape (oumuamua_bringup/config/task_commander.yaml, under the task_commander node). The bridge
# declares them under the same names but they are optional: without them the map draws a circle.
ROBOT_SHAPE_KEYS = ("footprint", "grippers.suction_offsets")

# Optional like ROBOT_SHAPE_KEYS, but with defaults (DEFAULTS above), so a controller_bridge.yaml that
# predates harite still starts. harite_speed is clamped to 0..100 rad/s (harite.yaml max_velocity).
HARITE_KEYS = ("harite_enabled", "harite_speed", "harite_reversed")


def robot_shape_from_text(text: str) -> dict:
    """The ROBOT_SHAPE_KEYS found in a task_commander.yaml text (`suction_offsets` sits under `grippers:`).
    bridge.launch.py uses it so the robot side needs no change to the bridge's own files."""
    flat = parse_simple_yaml(text)
    out = {}
    if isinstance(flat.get("footprint"), list):
        out["footprint"] = flat["footprint"]
    if isinstance(flat.get("suction_offsets"), list):
        out["grippers.suction_offsets"] = flat["suction_offsets"]
    return out


def from_ros_params(p: dict) -> dict:
    """ROS parameters (the bridge's own keys plus FIELD_KEYS) -> the flat keys build_config() takes.
    The obstacles shown are the walls and the shelf; the counts come from the field's lists."""
    out = {k: v for k, v in p.items() if not k.startswith("field.")}
    out["field_bounds"] = list(p["field.bounds"])
    out["field_obstacles"] = list(p["field.walls"]) + list(p["field.shelf"])
    out["shelf"] = list(p["field.shelf"])
    out["start_pose"] = list(p["field.start_pose"])
    out["blocks"] = len(p["field.notes_x"])
    out["columns"] = len(p["field.shelf_columns_x"])
    out["rows"] = int(p["field.shelf_rows"])
    out.pop("grippers.suction_offsets", None)
    if p.get("footprint") is not None:
        out["footprint"] = list(p["footprint"])
    if p.get("grippers.suction_offsets") is not None:
        out["suction_offsets"] = list(p["grippers.suction_offsets"])
    return out


def build_config(p: dict) -> dict:
    names, heights = list(p["lift_preset_names"]), list(p["lift_preset_heights"])
    sp = [float(v) for v in p["speed_levels"]]
    return {
        "robot_name": str(p["robot_name"]),
        "lift_presets": [[str(n), float(h)] for n, h in zip(names, heights)],
        "lift_max": float(p["lift_max"]),
        "speed_levels": [[sp[i], sp[i + 1]] for i in range(0, len(sp) - 1, 2)],
        "blocks": int(p["blocks"]), "columns": int(p["columns"]), "rows": int(p["rows"]),
        "field_bounds": [float(v) for v in p["field_bounds"]],
        "field_obstacles": [float(v) for v in p["field_obstacles"]],
        "shelf": [float(v) for v in p["shelf"]],
        "start_pose": [float(v) for v in p["start_pose"]],
        "robot_radius": float(p["robot_radius"]),
        "harite_enabled": bool(p.get("harite_enabled", DEFAULTS["harite_enabled"])),
        "harite_speed": clamp_speed(p.get("harite_speed", DEFAULTS["harite_speed"])),
        "harite_reversed": bool(p.get("harite_reversed", DEFAULTS["harite_reversed"])),
        "footprint": [float(v) for v in p.get("footprint") or []],
        "suction_offsets": [float(v) for v in p.get("suction_offsets") or []],
        "completion_setup_files": [str(s) for s in p["completion_setup_files"]],
    }


def expand_env(config: dict) -> dict:
    """Expand $VARS in completion_setup_files (done at use, not stored in the config file)."""
    out = dict(config)
    out["completion_setup_files"] = [os.path.expandvars(s) for s in config["completion_setup_files"]]
    return out


def parse_simple_yaml(text: str) -> dict:
    """Read `key: value` lines under the single `ros__parameters:` block of bridge.yaml.
    Values are scalars or (possibly multi-line) flow lists; `#` starts a comment."""
    params, buf, key = {}, "", None
    for raw in text.splitlines():
        line = raw.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        if key is not None:
            buf += " " + line.strip()
        else:
            k, sep, v = line.strip().partition(":")
            if not sep or not v.strip():
                continue  # structural line (bridge:, ros__parameters:)
            key, buf = k.strip(), v.strip()
        if buf.count("[") == buf.count("]"):
            try:
                params[key] = ast.literal_eval(buf)
            except (ValueError, SyntaxError):
                params[key] = buf  # a bare string (a topic name, ...)
            key = None
    return params
