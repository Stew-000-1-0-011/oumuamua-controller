from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    here = Path(__file__).resolve().parent
    default_config = here.parent / "config" / "bridge.yaml"
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory("oumuamua_controller_bridge"))
        default_config = share / "config" / "bridge.yaml"
    except Exception:
        pass
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=str(default_config)),
        Node(package="oumuamua_controller_bridge", executable="bridge", name="bridge",
             output="screen", parameters=[LaunchConfiguration("config")]),
    ])
