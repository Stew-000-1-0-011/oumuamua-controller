"""ROS-facing interface used by the supervisor and handlers (no rclpy here)."""
from dataclasses import dataclass, field
from typing import Callable, Optional, Protocol

from . import protocol as P

Done = Callable[[bool, str], None]


@dataclass
class Snapshot:
    pose: Optional[tuple] = None  # (x, y, yaw)
    vel: Optional[tuple] = None  # (vx, vy, wz)
    # Motors: the three wheels, then the two suction motors.
    # Each (vdc, ok, current [A], velocity [rad/s], mode, flags); see protocol.motor_row.
    wheels: list = field(default_factory=list)
    grippers: list = field(default_factory=lambda: [None, None])
    scan: list = field(default_factory=list)  # [(x, y)]
    path: list = field(default_factory=list)  # [(x, y)]
    task_text: Optional[str] = None
    tracker_enabled: bool = False


def motor_fields(m) -> tuple:
    """Plain values of a mini_shirasu_ros/msg/Status: (vdc, current, velocity, mode, flags)."""
    flags = ((P.SATURATED_UPPER if m.saturated_upper else 0)
             | (P.SATURATED_LOWER if m.saturated_lower else 0)
             | (P.VDC_LOW if m.vdc_low else 0))
    return float(m.vdc), float(m.current), float(m.velocity), int(m.mode), flags


def motor_entry(fields: Optional[tuple], fresh: bool) -> tuple:
    """Snapshot entry from `motor_fields` (None = nothing received yet). `ok` means the status
    is arriving and the board is not in fault."""
    if fields is None:
        return (0.0, False, 0.0, 0.0, P.MODE_UNKNOWN, 0)
    vdc, current, velocity, mode, flags = fields
    return (vdc, fresh and mode != P.MODE_FAULT, current, velocity, mode, flags)


class RosIo(Protocol):
    def publish_cmd_vel(self, vx: float, vy: float, wz: float) -> None: ...
    def stop_path(self, done: Done) -> None: ...
    def start_path(self, name: str, started: Done, done: Done) -> None:
        """Follow the named path (action). `started` fires when the goal is accepted; `done` fires
        once when the goal ends (success, failure, cancel) or was never accepted."""
        ...
    def set_tracker(self, on: bool, done: Done) -> None: ...
    def set_pose(self, x: float, y: float, yaw: float) -> None: ...
    def run_task(self, kind: int, gripper: int, block: int, column: int, row: int,
                 step: Callable[[str], None], done: Done) -> None: ...
    def cancel_task(self) -> None: ...
    def task_reset(self, done: Done) -> None: ...
    def set_suction(self, gripper: int, on: bool, done: Done) -> None: ...
    def lift_to(self, gripper: int, height: float, done: Done) -> None: ...
    def lift_cancel(self, gripper: int) -> None: ...
    def lift_zero(self, gripper: int, done: Done) -> None: ...
    def snapshot(self) -> Snapshot: ...
    def path_names(self) -> list: ...
