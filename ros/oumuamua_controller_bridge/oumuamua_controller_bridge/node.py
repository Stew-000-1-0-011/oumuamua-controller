"""ROS entry point: parameters -> RosIoNode + BridgeServer, spun by a MultiThreadedExecutor."""
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from .config import FIELD_KEYS, build_config, expand_env, from_ros_params
from .ros_io import RosIoNode
from .server import BridgeServer

# Keys of the bridge's ROS parameters. There are no defaults: every key must be in the parameter files
# (oumuamua_bringup/config/controller_bridge.yaml and, for field.*, field.yaml).
# config.DEFAULTS is only for the fake robot and the tests.
PARAM_KEYS = ("robot_name", "lift_max", "robot_radius", "completion_setup_files",
              "lift_preset_names", "lift_preset_heights", "speed_levels") + FIELD_KEYS


def _read_params(node: Node) -> dict:
    any_type = ParameterDescriptor(dynamic_typing=True)
    for key in PARAM_KEYS:
        node.declare_parameter(key, None, any_type)
    params = {key: node.get_parameter(key).value for key in PARAM_KEYS}
    missing = [key for key, value in params.items() if value is None]
    if missing:
        raise RuntimeError("parameters not set (give them in the parameter file): " + ", ".join(missing))
    return params


def main(args=None):
    rclpy.init(args=args)
    io = RosIoNode()
    server = None
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        config = expand_env(build_config(from_ros_params(_read_params(io))))
        server = BridgeServer(config["robot_name"], io, config,
                              log_root=Path.home() / ".oumuamua_controller" / "console")
        io.notify = server.notify
        server.start()
        io.get_logger().info(f"bridge '{config['robot_name']}' on UDP {server.port}")
        executor.add_node(io)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
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
