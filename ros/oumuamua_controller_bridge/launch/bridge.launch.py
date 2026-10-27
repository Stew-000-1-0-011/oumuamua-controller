from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # The bridge has no parameter defaults. The parameter file lives with the robot's other settings
    # (oumuamua_bringup/config/controller_bridge.yaml); without oumuamua_bringup, config:= is required.
    config = DeclareLaunchArgument("config", description="parameter file of the bridge (required)")
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory("oumuamua_bringup")
        config = DeclareLaunchArgument("config", default_value=f"{share}/config/controller_bridge.yaml",
                                       description="parameter file of the bridge")
    except Exception:
        pass
    return LaunchDescription([
        config,
        Node(package="oumuamua_controller_bridge", executable="bridge", name="bridge",
             output="screen", parameters=[LaunchConfiguration("config")]),
    ])
