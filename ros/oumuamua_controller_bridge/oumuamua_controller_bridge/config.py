"""Bridge config helpers (no rclpy): flat ROS parameters -> BridgeServer config dict, and a
minimal reader for config/bridge.yaml so the tests need no PyYAML."""
import ast
import os


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
            params[key] = ast.literal_eval(buf)
            key = None
    return params
