from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # The bridge has no parameter defaults. Its parameter files live with the robot's other settings
    # (oumuamua_bringup/config/field.yaml for field.* and controller_bridge.yaml for the rest);
    # without oumuamua_bringup, field:= and config:= are required.
    field = DeclareLaunchArgument("field", description="field parameter file (field.*, required)")
    config = DeclareLaunchArgument("config", description="parameter file of the bridge (required)")
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory("oumuamua_bringup")
        field = DeclareLaunchArgument("field", default_value=f"{share}/config/field.yaml",
                                      description="field parameter file (field.*)")
        config = DeclareLaunchArgument("config", default_value=f"{share}/config/controller_bridge.yaml",
                                       description="parameter file of the bridge")
    except Exception:
        pass
    return LaunchDescription([
        field,
        config,
        Node(package="oumuamua_controller_bridge", executable="bridge", name="bridge", output="screen",
             parameters=[LaunchConfiguration("field"), LaunchConfiguration("config")]),
    ])
