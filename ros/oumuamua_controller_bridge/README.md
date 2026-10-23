# oumuamua_controller_bridge

UDP/MessagePack bridge between the oumuamua Android controller and ROS 2 (wire format:
`docs/protocol.md`). Everything except `ros_io.py` and `node.py` is ROS-free.

## Run with ROS 2

Build together with the oumuamua workspace (depends on `gripper`, `task_commander`, `path_follower`,
`mini_shirasu_ros`):

    colcon build --packages-up-to oumuamua_controller_bridge
    ros2 launch oumuamua_controller_bridge bridge.launch.py [config:=/path/to/bridge.yaml]

Parameters are in `config/bridge.yaml` (lists of pairs are flattened because ROS parameters cannot
nest lists; `config.build_config()` rebuilds them). `$ROS_DISTRO` in `completion_setup_files` is
expanded at start-up.

## Run without ROS (simulated robot)

    python3 -m oumuamua_controller_bridge.fake_robot [--port 47800]

Runs `BridgeServer` on `FakeRosIo` (planar cmd_vel integration, 1 s per task stage, lift 0.05 m/s,
LiDAR against the outer walls). Its `DEFAULT_CONFIG` mirrors `config/bridge.yaml`;
`test_fake_robot.py::test_default_config_matches_bridge_yaml` keeps them equal.

## Tests

    python3 -m venv .venv && .venv/bin/pip install msgpack pytest
    .venv/bin/pytest ros/oumuamua_controller_bridge/test

## Please verify on a machine with ROS 2 (not checked here; only `py_compile`)

- `colcon build` of the package succeeds; `ros2 launch ... bridge.launch.py` starts and the node
  prints its UDP port.
- With `sim:=true` bringup: the phone connects, STATE shows pose/wheels/grippers, SCAN draws points
  in the `field` frame (needs TF `field` <- scan frame), `/cmd_vel` moves the sim robot.
- Path list in CONFIG matches `ros2 service list | grep /path_follower/start/`; start/stop/tracker work.
- `RunTask` goals: feedback steps show up, cancel and failures end with TASK_DONE(false, reason).
- MoveLift goals and cancel; suction; `/gripper{i}/zero_lift` (assumed to exist; without it the
  phone gets "<name> がありません" after 1 s).
- `/rosout` lines arrive as LOG messages.
