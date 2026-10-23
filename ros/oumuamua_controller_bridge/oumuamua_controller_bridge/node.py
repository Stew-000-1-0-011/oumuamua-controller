"""ROS entry point: parameters -> RosIoNode + BridgeServer, spun by a MultiThreadedExecutor."""
from pathlib import Path

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from .config import build_config, expand_env
from .fake_robot import DEFAULT_CONFIG
from .ros_io import RosIoNode
from .server import BridgeServer

def _flat_defaults() -> dict:
    """Same keys as config/bridge.yaml, defaults from fake_robot.DEFAULT_CONFIG (ROS-free)."""
    c = DEFAULT_CONFIG
    flat = {k: c[k] for k in ("robot_name", "lift_max", "blocks", "columns", "rows",
                              "field_bounds", "field_obstacles", "shelf", "start_pose",
                              "robot_radius", "completion_setup_files")}
    flat["lift_preset_names"] = [n for n, _ in c["lift_presets"]]
    flat["lift_preset_heights"] = [h for _, h in c["lift_presets"]]
    flat["speed_levels"] = [v for pair in c["speed_levels"] for v in pair]
    return flat


PARAM_DEFAULTS = _flat_defaults()


def _read_params(node: Node) -> dict:
    for key, default in PARAM_DEFAULTS.items():
        node.declare_parameter(key, default)
    return {key: node.get_parameter(key).value for key in PARAM_DEFAULTS}


def main(args=None):
    rclpy.init(args=args)
    io = RosIoNode()
    server = None
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        config = expand_env(build_config(_read_params(io)))
        server = BridgeServer(config["robot_name"], io, config,
                              log_root=Path.home() / ".oumuamua_controller" / "console")
        io.notify = server.notify
        server.start()
        io.get_logger().info(f"bridge '{config['robot_name']}' on UDP {server.port}")
        executor.add_node(io)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        io.notify = None
        if server is not None:
            server.stop()
        executor.shutdown()
        io.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
