from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _bridge(context):
    from pathlib import Path

    from oumuamua_controller_bridge.config import robot_shape_from_text

    # The robot's shape (footprint, grippers.suction_offsets) is read out of task_commander.yaml here, because
    # that file's keys sit under the task_commander node and ROS would not give them to the bridge. Keys in the
    # bridge's own file (config:=) come later and win.
    shape = {}
    path = Path(LaunchConfiguration("robot").perform(context))
    if path.is_file():
        shape = robot_shape_from_text(path.read_text(encoding="utf-8"))
    params = [LaunchConfiguration("field")] + ([shape] if shape else []) + [LaunchConfiguration("config")]
    return [Node(package="oumuamua_controller_bridge", executable="bridge", name="bridge", output="screen",
                 parameters=params)]


def generate_launch_description():
    # The bridge has no parameter defaults. Its parameter files live with the robot's other settings
    # (oumuamua_bringup/config/field.yaml for field.* and controller_bridge.yaml for the rest);
    # without oumuamua_bringup, field:= and config:= are required. robot:= (task_commander.yaml) is optional:
    # without it the map draws a circle of robot_radius instead of the footprint.
    field = DeclareLaunchArgument("field", description="field parameter file (field.*, required)")
    config = DeclareLaunchArgument("config", description="parameter file of the bridge (required)")
    robot = DeclareLaunchArgument("robot", default_value="", description="task_commander.yaml (footprint, suction)")
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory("oumuamua_bringup")
        field = DeclareLaunchArgument("field", default_value=f"{share}/config/field.yaml",
                                      description="field parameter file (field.*)")
        config = DeclareLaunchArgument("config", default_value=f"{share}/config/controller_bridge.yaml",
                                       description="parameter file of the bridge")
        robot = DeclareLaunchArgument("robot", default_value=f"{share}/config/task_commander.yaml",
                                      description="task_commander.yaml (footprint, grippers.suction_offsets)")
    except Exception:
        pass
    return LaunchDescription([field, config, robot, OpaqueFunction(function=_bridge)])
