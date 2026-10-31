"""rclpy implementation of RosIo. Only this module and node.py import ROS.

Threading: the node is spun by a MultiThreadedExecutor and every client/subscription uses one
ReentrantCallbackGroup. `done` / `step` callbacks may run on an executor thread (or, when a service
is missing, on a short-lived helper thread); BridgeServer protects itself with the supervisor lock.
No internal lock is ever held while a `done` / `step` callback runs, because those callbacks take
the supervisor lock and the supervisor calls back into us while holding it.
"""
import math
import threading
from typing import Callable, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from gripper.action import MoveLift
from gripper.msg import GripperState
from mini_shirasu_ros.msg import Status as MotorStatus
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMsg
from path_follower.action import FollowPath
from rcl_interfaces.msg import Log
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, String
from std_srvs.srv import SetBool, Trigger
from task_commander.action import RunTask
from tf2_ros import Buffer, TransformListener

from . import protocol as P
from .guards import REPLY_TIMEOUT_S, Watchdog, once, with_timeout
from .harite import command_value
from .rosio import Done, Snapshot, floor_rotation, motor_entry, motor_fields

WHEEL_COUNT = P.WHEEL_COUNT
SUCTION_COUNT = P.SUCTION_COUNT
MOTOR_COUNT = WHEEL_COUNT + SUCTION_COUNT
GRIPPER_COUNT = 2
WHEEL_TIMEOUT_S = 0.5
SERVICE_WAIT_S = 1.0
FIELD_FRAME = "field"
BASE_FRAME = "base_link"
START_PREFIX = "/path_follower/start/"
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
RELIABLE_10 = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
ROSOUT_QOS = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)


def _yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _noop(ok, msg):
    pass


class _Goal:
    """One in-flight action goal: lets a cancel arrive before the goal is accepted."""

    def __init__(self):
        self.handle = None
        self.cancel_requested = False
        self.finished = False
        self.done = None  # once-guarded done, set by _send_goal
        self.cancel_dog = None


class RosIoNode(Node):
    """Implements rosio.RosIo on rclpy. `notify` is set by node.py once BridgeServer exists."""

    def __init__(self):
        super().__init__("bridge")
        self.notify: Optional[Callable[[list], None]] = None
        self._cbg = ReentrantCallbackGroup()
        self._lock = threading.Lock()  # guards the data below only
        self._pose = None
        self._vel = None
        # Mini-shirasu boards: wheels 0..2, then suction 0..1: (motor_fields, Time) or None.
        self._motors = [None] * MOTOR_COUNT
        self._grippers = [None] * GRIPPER_COUNT
        self._scan = []
        self._path = []
        self._task_text = None
        self._tracker_enabled = False
        self._harite_command = None
        self._task_goal: Optional[_Goal] = None
        self._lift_goals = [None] * GRIPPER_COUNT
        self._follow_goal: Optional[_Goal] = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        g = self._cbg
        self._cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self._pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self._harite_pub = self.create_publisher(Float64, "/harite/target_velocity", 10)

        self.create_subscription(Odometry, "/state_estimator/odom", self._on_odom, 10,
                                 callback_group=g)
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data,
                                 callback_group=MutuallyExclusiveCallbackGroup())
        self.create_subscription(PathMsg, "/path_follower/path", self._on_path, LATCHED,
                                 callback_group=g)
        self.create_subscription(String, "/task_commander/state", self._on_task_state, LATCHED,
                                 callback_group=g)
        self.create_subscription(Float64, "/harite/command", self._on_harite_command, 10,
                                 callback_group=g)
        self.create_subscription(Log, "/rosout", self._on_rosout, ROSOUT_QOS, callback_group=g)
        topics = ([f"/wheel{i}/status" for i in range(WHEEL_COUNT)]
                  + [f"/suction{i}/status" for i in range(SUCTION_COUNT)])
        for i, topic in enumerate(topics):
            self.create_subscription(MotorStatus, topic,
                                     lambda m, i=i: self._on_motor(i, m), 10, callback_group=g)
        for i in range(GRIPPER_COUNT):
            self.create_subscription(GripperState, f"/gripper{i}/state",
                                     lambda m, i=i: self._on_gripper(i, m), 10, callback_group=g)

        self._stop_cli = self.create_client(Trigger, "/path_follower/stop", callback_group=g)
        self._reset_cli = self.create_client(Trigger, "/task_commander/reset", callback_group=g)
        self._tracker_cli = self.create_client(SetBool, "/tracker_node/enable", callback_group=g)
        self._harite_cli = self.create_client(SetBool, "/harite/enable", callback_group=g)
        self._suction_cli = [self.create_client(SetBool, f"/gripper{i}/set_suction",
                                                callback_group=g) for i in range(GRIPPER_COUNT)]
        self._zero_cli = [self.create_client(Trigger, f"/gripper{i}/zero_lift", callback_group=g)
                          for i in range(GRIPPER_COUNT)]
        self._run_ac = ActionClient(self, RunTask, "/task_commander/run", callback_group=g)
        self._follow_ac = ActionClient(self, FollowPath, "/path_follower/follow", callback_group=g)
        self._lift_ac = [ActionClient(self, MoveLift, f"/gripper{i}/move_lift", callback_group=g)
                         for i in range(GRIPPER_COUNT)]

    # -- subscriptions --------------------------------------------------
    def _on_odom(self, m: Odometry):
        p, t = m.pose.pose, m.twist.twist
        with self._lock:
            self._pose = (p.position.x, p.position.y, _yaw_of(p.orientation))
            self._vel = (t.linear.x, t.linear.y, t.angular.z)

    def _on_motor(self, i: int, m):
        fields = motor_fields(m)
        with self._lock:
            self._motors[i] = (fields, self.get_clock().now())

    def _on_gripper(self, i: int, m):
        with self._lock:
            self._grippers[i] = (m.lift_height, m.lift_goal, m.lift_moving, m.lift_arrived,
                                 m.suction_on, m.lift_feedback)

    def _on_harite_command(self, m: Float64):
        with self._lock:
            self._harite_command = command_value(m.data)

    def _on_path(self, m: PathMsg):
        pts = [(ps.pose.position.x, ps.pose.position.y) for ps in m.poses]
        with self._lock:
            self._path = pts

    def _on_task_state(self, m: String):
        with self._lock:
            self._task_text = m.data

    def _on_rosout(self, m: Log):
        notify = self.notify
        if notify is None or m.name == self.get_name():
            return
        stamp_ms = int(m.stamp.sec) * 1000 + int(m.stamp.nanosec) // 1_000_000
        notify([P.LOG, stamp_ms, int(m.level), str(m.name), str(m.msg)])

    def _on_scan(self, m: LaserScan):
        """Ranges -> `field` frame (every 2nd valid point). No TF yet: publish no points.
        Uses the whole rotation, not just the yaw: the LiDAR is mounted upside down (roll pi)."""
        tf = None
        for stamp in (Time.from_msg(m.header.stamp), Time()):  # at scan time, then latest
            try:
                tf = self._tf_buffer.lookup_transform(FIELD_FRAME, m.header.frame_id, stamp,
                                                      timeout=Duration(seconds=0.0))
                break
            except Exception:
                continue
        if tf is None:
            with self._lock:
                self._scan = []
            return
        t = tf.transform
        xx, xy, yx, yy = floor_rotation(t.rotation)
        tx, ty = t.translation.x, t.translation.y
        pts = []
        for k in range(0, len(m.ranges), 2):
            r = m.ranges[k]
            if not math.isfinite(r) or r < m.range_min or r > m.range_max:
                continue
            a = m.angle_min + k * m.angle_increment
            x, y = r * math.cos(a), r * math.sin(a)
            pts.append((tx + xx * x + xy * y, ty + yx * x + yy * y))
        with self._lock:
            self._scan = pts

    # -- RosIo: publishing ----------------------------------------------
    def publish_cmd_vel(self, vx: float, vy: float, wz: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = BASE_FRAME
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = vx, vy, wz
        self._cmd_pub.publish(msg)

    def publish_harite(self, velocity: float) -> None:
        msg = Float64()
        msg.data = float(velocity)
        self._harite_pub.publish(msg)

    def set_pose(self, x: float, y: float, yaw: float) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = FIELD_FRAME
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        cov = [0.0] * 36
        cov[0] = cov[7] = cov[35] = 0.01  # x, y, yaw
        msg.pose.covariance = cov
        self._pose_pub.publish(msg)

    # -- RosIo: services ------------------------------------------------
    def _call(self, client, request, name: str, done: Done, on_ok=None) -> None:
        """Call a service; done(False, "<name> がありません") if it is not up within 1 s, and
        done(False, "<name>: 応答がありません") if it does not reply within 5 s.
        Never blocks the caller (the supervisor calls us under its lock)."""
        done = once(done)
        if client.service_is_ready():
            self._send_request(client, request, name, done, on_ok)
            return

        def wait_then_call():
            if client.wait_for_service(timeout_sec=SERVICE_WAIT_S):
                self._send_request(client, request, name, done, on_ok)
            else:
                done(False, f"{name} がありません")

        threading.Thread(target=wait_then_call, name="svc-wait", daemon=True).start()

    def _send_request(self, client, request, name, done, on_ok) -> None:
        done, _dog = with_timeout(done, f"{name}: 応答がありません")
        try:
            future = client.call_async(request)
        except Exception as e:  # e.g. context shut down
            done(False, f"{name}: {e}")
            return

        def finished(fut):
            try:
                res = fut.result()
                ok, msg = bool(res.success), str(res.message)
            except Exception as e:
                done(False, f"{name}: {e}")
                return
            done(ok, msg)  # first, so a failing follow-up cannot swallow it
            if ok and on_ok:
                try:
                    on_ok()
                except Exception as e:
                    self.get_logger().error(f"{name}: {e}")

        future.add_done_callback(finished)

    def stop_path(self, done: Done) -> None:
        with self._lock:
            goal = self._follow_goal
        self._cancel_goal(goal)  # the follow goal's done fires (False) when it is cancelled
        self._call(self._stop_cli, Trigger.Request(), "/path_follower/stop", done)

    def start_path(self, name: str, started: Done, done: Done) -> None:
        """Run the named path with the /path_follower/follow action: `started` on acceptance,
        `done` when the goal ends (or is rejected / cancelled)."""
        goal_msg = FollowPath.Goal()
        goal_msg.route = name
        goal = _Goal()
        with self._lock:
            self._follow_goal = goal

        def result(res):
            return bool(res.success), str(res.message)

        self._send_goal(self._follow_ac, goal_msg, "/path_follower/follow", goal, None, result,
                        done, lambda: started(True, ""))

    def set_tracker(self, on: bool, done: Done) -> None:
        req = SetBool.Request()
        req.data = bool(on)

        def remember():
            with self._lock:
                self._tracker_enabled = bool(on)

        self._call(self._tracker_cli, req, "/tracker_node/enable", done, remember)

    def set_harite_enabled(self, on: bool, done: Done) -> None:
        req = SetBool.Request()
        req.data = bool(on)
        self._call(self._harite_cli, req, "/harite/enable", done)

    def task_reset(self, done: Done) -> None:
        self._call(self._reset_cli, Trigger.Request(), "/task_commander/reset", done)

    def set_suction(self, gripper: int, on: bool, done: Done) -> None:
        req = SetBool.Request()
        req.data = bool(on)
        self._call(self._suction_cli[gripper], req, f"/gripper{gripper}/set_suction", done)

    def lift_zero(self, gripper: int, done: Done) -> None:
        # /gripper{i}/zero_lift is assumed to exist (added on the oumuamua side).
        self._call(self._zero_cli[gripper], Trigger.Request(), f"/gripper{gripper}/zero_lift",
                   done)

    # -- RosIo: actions -------------------------------------------------
    def _send_goal(self, client, goal_msg, name: str, goal: _Goal, on_feedback, on_result,
                   done: Done, on_accept=None) -> None:
        """Send an action goal without blocking. `on_result(result)` -> (ok, message) is used for
        a finished goal; rejection, abort, cancel, errors and a silent server all end in done().
        Acceptance has a 5 s watchdog; the running goal itself has no timeout."""
        if goal.done is None:
            goal.done = once(done)
        done = goal.done
        if not client.server_is_ready():
            def wait_then_send():
                if client.wait_for_server(timeout_sec=SERVICE_WAIT_S):
                    self._send_goal(client, goal_msg, name, goal, on_feedback, on_result, done,
                                    on_accept)
                else:
                    goal.finished = True
                    done(False, f"{name} がありません")

            threading.Thread(target=wait_then_send, name="action-wait", daemon=True).start()
            return

        def accept_timeout():
            goal.finished = True
            goal.cancel_requested = True  # if it is accepted late, cancel it
            done(False, f"{name}: 応答がありません")

        accept_dog = Watchdog(REPLY_TIMEOUT_S, accept_timeout)

        def accepted(fut):
            accept_dog.cancel()
            try:
                handle = fut.result()
            except Exception as e:
                goal.finished = True
                done(False, f"{name}: {e}")
                return
            if not handle.accepted:
                goal.finished = True
                done(False, f"{name}: 受け付けられませんでした")
                return
            goal.handle = handle
            if goal.cancel_requested:
                handle.cancel_goal_async()
            if goal.finished:  # timed out already: just cancel, the result is not awaited
                return
            handle.get_result_async().add_done_callback(finished)
            if on_accept is not None:
                try:
                    on_accept()
                except Exception as e:
                    self.get_logger().error(f"{name}: {e}")

        def finished(fut):
            goal.finished = True
            if goal.cancel_dog:
                goal.cancel_dog.cancel()
            try:
                wrapped = fut.result()
                if wrapped.status == GoalStatus.STATUS_CANCELED:
                    done(False, "取り消されました")
                elif wrapped.status == GoalStatus.STATUS_ABORTED:
                    done(False, getattr(wrapped.result, "message", "") or "中断されました")
                else:
                    ok, msg = on_result(wrapped.result)
                    done(ok, msg)
            except Exception as e:
                done(False, f"{name}: {e}")

        try:
            client.send_goal_async(goal_msg, feedback_callback=on_feedback).add_done_callback(
                accepted)
        except Exception as e:
            accept_dog.cancel()
            goal.finished = True
            done(False, f"{name}: {e}")

    @staticmethod
    def _cancel_goal(goal: Optional[_Goal]) -> None:
        """Cancel; the result callback normally resolves done. If the server stays silent for
        5 s, done(False) fires anyway."""
        if goal is None or goal.finished or goal.cancel_requested:
            return
        goal.cancel_requested = True
        if goal.done is not None and goal.cancel_dog is None:
            d = goal.done

            def timeout():
                goal.finished = True
                d(False, "取り消しの応答がありません")

            goal.cancel_dog = Watchdog(REPLY_TIMEOUT_S, timeout)
        if goal.handle is not None:
            goal.handle.cancel_goal_async()

    def run_task(self, kind: int, gripper: int, block: int, column: int, row: int,
                 step: Callable[[str], None], done: Done) -> None:
        goal_msg = RunTask.Goal()
        goal_msg.type = int(kind)
        goal_msg.gripper, goal_msg.block = int(gripper), int(block)
        goal_msg.column, goal_msg.row = int(column), int(row)
        goal = _Goal()
        with self._lock:
            self._task_goal = goal

        def feedback(fb):
            step(str(fb.feedback.step))

        def result(res):
            return bool(res.success), str(res.message)

        self._send_goal(self._run_ac, goal_msg, "/task_commander/run", goal, feedback, result,
                        done)

    def cancel_task(self) -> None:
        with self._lock:
            goal = self._task_goal
        self._cancel_goal(goal)

    def lift_to(self, gripper: int, height: float, done: Done) -> None:
        goal_msg = MoveLift.Goal()
        goal_msg.height = float(height)
        goal = _Goal()
        with self._lock:
            self._lift_goals[gripper] = goal

        def result(res):
            return bool(res.reached), str(res.message)

        self._send_goal(self._lift_ac[gripper], goal_msg, f"/gripper{gripper}/move_lift", goal,
                        None, result, done)

    def lift_cancel(self, gripper: int) -> None:
        with self._lock:
            goal = self._lift_goals[gripper]
        self._cancel_goal(goal)

    # -- RosIo: state ---------------------------------------------------
    def snapshot(self) -> Snapshot:
        now = self.get_clock().now()
        with self._lock:
            wheels = []
            for w in self._motors:
                if w is None:
                    wheels.append(motor_entry(None, False))
                    continue
                fields, stamp = w
                fresh = (now - stamp).nanoseconds * 1e-9 <= WHEEL_TIMEOUT_S
                wheels.append(motor_entry(fields, fresh))
            return Snapshot(pose=self._pose, vel=self._vel, wheels=wheels,
                            grippers=list(self._grippers), scan=list(self._scan),
                            path=list(self._path), task_text=self._task_text,
                            tracker_enabled=self._tracker_enabled,
                            harite_command=self._harite_command)

    def path_names(self) -> list:
        names = []
        for name, _types in self.get_service_names_and_types():
            if name.startswith(START_PREFIX) and len(name) > len(START_PREFIX):
                names.append(name[len(START_PREFIX):])
        return sorted(names)
