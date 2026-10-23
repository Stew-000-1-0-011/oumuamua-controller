"""Recording fake of RosIo. Dones fire immediately unless auto_done is False."""
from oumuamua_controller_bridge.rosio import Snapshot


class FakeRosIo:
    def __init__(self, auto_done=True):
        self.calls = []
        self.auto_done = auto_done
        self.pending = []  # (name, done)
        self.fail = {}  # name -> message: make that call report failure
        self.task_step = None
        self.task_done = None
        self.snap = Snapshot()
        self.names = []
        self.path_done = None
        self.on_run_task = None  # hook called at the start of run_task

    def _finish(self, name, done):
        if self.auto_done:
            msg = self.fail.get(name)
            done(msg is None, msg or "")
        else:
            self.pending.append((name, done))

    def publish_cmd_vel(self, vx, vy, wz):
        self.calls.append(("cmd_vel", vx, vy, wz))

    def stop_path(self, done):
        self.calls.append(("stop_path",))
        self._finish("stop_path", done)

    def start_path(self, name, started, done):
        self.calls.append(("start_path", name))
        self.path_done = done  # call it to end the path
        msg = self.fail.get("start_path")
        if msg is not None:
            done(False, msg)
        else:
            self._finish("start_path", started)

    def set_tracker(self, on, done):
        self.calls.append(("set_tracker", on))
        self._finish("set_tracker", done)

    def set_pose(self, x, y, yaw):
        self.calls.append(("set_pose", x, y, yaw))

    def run_task(self, kind, gripper, block, column, row, step, done):
        if self.on_run_task:
            self.on_run_task()
        self.calls.append(("run_task", kind, gripper, block, column, row))
        self.task_step, self.task_done = step, done

    def cancel_task(self):
        self.calls.append(("cancel_task",))

    def task_reset(self, done):
        self.calls.append(("task_reset",))
        self._finish("task_reset", done)

    def set_suction(self, gripper, on, done):
        self.calls.append(("set_suction", gripper, on))
        self._finish(("set_suction", gripper), done)

    def lift_to(self, gripper, height, done):
        self.calls.append(("lift_to", gripper, height))
        self._finish(("lift_to", gripper), done)

    def lift_cancel(self, gripper):
        self.calls.append(("lift_cancel", gripper))

    def lift_zero(self, gripper, done):
        self.calls.append(("lift_zero", gripper))
        self._finish(("lift_zero", gripper), done)

    def snapshot(self):
        return self.snap

    def path_names(self):
        return list(self.names)

    def non_vel(self):
        return [c for c in self.calls if c[0] != "cmd_vel"]
