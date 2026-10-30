"""ROS-free simulated robot: a RosIo implementation plus `main()` that runs BridgeServer on it.

    python3 -m oumuamua_controller_bridge.fake_robot [--port 47800]

Simplifications: cmd_vel is integrated on a plane (no wall collisions); a path run is accepted at once and finishes after 3 s; a task walks through its stages one second each and then succeeds; lifts move at
0.05 m/s; suction switches instantly; the LiDAR casts a ray every 1 degree against the outer field
walls only (obstacles are not hit). The model advances whenever snapshot() is called.
"""
import argparse
import math
import threading
import time
from pathlib import Path

from . import protocol as P
from .config import DEFAULTS, expand_env
from .rosio import Snapshot

DEFAULT_CONFIG = DEFAULTS

LIFT_SPEED = 0.05  # m/s
STAGE_S = 1.0
PATH_RUN_S = 3.0
MAX_RANGE = 30.0
# Motor model (the demo crate mirrors it): wheel i sits at 90 + 120 i degrees; its speed is the
# commanded body velocity projected on the rolling direction, plus the turn. Current is
# proportional to speed; a suction motor draws SUCTION_A while on. The bus sags with the load.
WHEEL_RADIUS = 0.05  # m
WHEEL_BASE = 0.2  # m, centre to wheel
WHEEL_A_PER_RAD_S = 0.06
SUCTION_A = 2.5
SUCTION_RAD_S = 300.0
VDC_NOMINAL = 24.0
VDC_SAG_PER_A = 0.05


def motor_snapshot(cmd, suction):
    """Five motors (3 wheels, 2 suction) for the commanded body velocity and suction states."""
    vx, vy, wz = cmd
    out = []
    for i in range(P.WHEEL_COUNT):
        a = math.radians(90.0 + 120.0 * i)
        speed = (-math.sin(a) * vx + math.cos(a) * vy + WHEEL_BASE * wz) / WHEEL_RADIUS
        cur = WHEEL_A_PER_RAD_S * speed
        out.append((VDC_NOMINAL - VDC_SAG_PER_A * abs(cur), True, cur, speed, P.MODE_VELOCITY, 0))
    for on in suction:
        if on:
            out.append((VDC_NOMINAL - VDC_SAG_PER_A * SUCTION_A, True, SUCTION_A, SUCTION_RAD_S,
                        P.MODE_VELOCITY, 0))
        else:
            out.append((VDC_NOMINAL, True, 0.0, 0.0, P.MODE_DISABLED, 0))
    return out


PATHS = ["slalom_forward", "slalom_backward"]
# 0 PICK, 5 TO_SHELF_CELL, 7 TO_SHELF_FRONT, 8 PICK_PAIR (task_commander's RunTask.action). 5 and 7 only
# move: the lift, the suction and what is held are not touched (the operator does those by hand).
TASK_STAGES = {0: ["ブロックへ移動", "吸着", "持ち上げ"], 5: ["マスへ移動"], 7: ["棚の前へ移動"],
               8: ["ブロックへ移動", "吸着", "持ち上げ"]}
# PICK_PAIR blocks that have a pose in task_commander.yaml pick.pairs (gripper0 takes block, gripper1 block - 1).
PAIR_BLOCKS = (4, 2)
DEFAULT_STAGES = ["移動中", "到着"]


def _ray_box_exit(ox, oy, dx, dy, box):
    """Distance from an interior point to the box boundary along (dx, dy), or None."""
    x0, y0, x1, y1 = box
    best = None
    for o, d, lo, hi in ((ox, dx, x0, x1), (oy, dy, y0, y1)):
        if abs(d) < 1e-12:
            continue
        t = ((hi if d > 0 else lo) - o) / d
        if t >= 0 and (best is None or t < best):
            best = t
    return best


class FakeRosIo:
    def __init__(self, config=None, clock=time.monotonic):
        self.config = dict(config or DEFAULT_CONFIG)
        self._clock = clock
        self._lock = threading.RLock()
        self._pose = list(self.config["start_pose"])
        self._cmd = (0.0, 0.0, 0.0)
        self._last = clock()
        self._tracker = False
        self._suction = [False, False]
        self._lift = [dict(h=0.0, goal=0.0, moving=False, arrived=True, done=None) for _ in range(2)]
        self._task = None  # dict(kind, gripper, block, column, row, stages, i, t0, step, done)
        self._holding = [False, False]
        self._picked = []
        self._path = []
        self._follow = None  # dict(t0, done) while a path run is active

    # -- model ----------------------------------------------------------
    def _advance(self):
        """Step the model to now; returns the callbacks to fire after the lock is released."""
        fire = []
        now = self._clock()
        dt = max(0.0, now - self._last)
        self._last = now
        vx, vy, wz = self._cmd
        x, y, yaw = self._pose
        c, s = math.cos(yaw), math.sin(yaw)
        self._pose = [x + (vx * c - vy * s) * dt, y + (vx * s + vy * c) * dt, yaw + wz * dt]
        for lift in self._lift:
            if not lift["moving"]:
                continue
            err = lift["goal"] - lift["h"]
            step = LIFT_SPEED * dt
            if abs(err) <= step:
                lift.update(h=lift["goal"], moving=False, arrived=True)
                done, lift["done"] = lift["done"], None
                if done:
                    fire.append(lambda d=done: d(True, ""))
            else:
                lift["h"] += math.copysign(step, err)
        t = self._task
        if t is not None:
            while t is self._task and now - t["t0"] >= STAGE_S:
                t["t0"] += STAGE_S
                t["i"] += 1
                if t["i"] >= len(t["stages"]):
                    self._finish_task_effects(t)
                    self._task = None
                    fire.append(lambda d=t["done"]: d(True, ""))
                else:
                    fire.append(lambda f=t["step"], s=t["stages"][t["i"]]: f(s))
        f = self._follow
        if f is not None and now - f["t0"] >= PATH_RUN_S:
            self._follow, self._path = None, []
            fire.append(lambda d=f["done"]: d(True, ""))
        return fire

    def _finish_task_effects(self, t):
        g = t["gripper"] if t["gripper"] in (0, 1) else 0
        if t["kind"] == 0:
            self._holding[g] = True
            self._picked.append(t["block"])
        elif t["kind"] == 8:
            self._holding = [True, True]
            self._picked += [t["block"], t["block"] - 1]

    @staticmethod
    def _run(fire):
        for f in fire:
            f()

    def _task_text(self):
        h = " ".join(f"gripper{i}={'yes' if v else 'no'}" for i, v in enumerate(self._holding))
        return f"holding: {h}\npicked blocks: {' '.join(str(b) for b in self._picked)}"

    def _scan(self):
        x, y, yaw = self._pose
        box = self.config["field_bounds"]
        pts = []
        if not (box[0] < x < box[2] and box[1] < y < box[3]):
            return pts
        for deg in range(360):
            a = yaw + math.radians(deg)
            d = _ray_box_exit(x, y, math.cos(a), math.sin(a), box)
            if d is not None and d <= MAX_RANGE:
                pts.append((x + d * math.cos(a), y + d * math.sin(a)))
        return pts

    # -- RosIo ----------------------------------------------------------
    def publish_cmd_vel(self, vx, vy, wz):
        with self._lock:
            self._cmd = (float(vx), float(vy), float(wz))

    def stop_path(self, done):
        with self._lock:
            f, self._follow = self._follow, None
            self._path = []
        if f:
            f["done"](False, "canceled")
        done(True, "")

    def start_path(self, name, started, done):
        with self._lock:
            prev, self._follow = self._follow, None
            x0, y0 = self._pose[0], self._pose[1]
            self._path = [(x0 + 0.1 * i, y0) for i in range(20)]
            self._follow = dict(t0=self._clock(), done=done)
        if prev:
            prev["done"](False, "aborted")
        started(True, "")

    def set_tracker(self, on, done):
        with self._lock:
            self._tracker = bool(on)
        done(True, "")

    def set_pose(self, x, y, yaw):
        with self._lock:
            self._pose = [float(x), float(y), float(yaw)]

    def run_task(self, kind, gripper, block, column, row, step, done):
        stages = TASK_STAGES.get(kind, DEFAULT_STAGES)
        if kind == 8 and block not in PAIR_BLOCKS:
            done(False, f"no pose for the pair of blocks {block} and {block - 1}")
            return
        with self._lock:
            if self._task is not None:
                reject = True
            else:
                reject = False
                self._task = dict(kind=kind, gripper=gripper, block=block, column=column, row=row,
                                  stages=stages, i=0, t0=self._clock(), step=step, done=done)
        if reject:
            done(False, "task already running")
        else:
            step(stages[0])

    def cancel_task(self):
        with self._lock:
            t, self._task = self._task, None
        if t:
            t["done"](False, "canceled")

    def task_reset(self, done):
        with self._lock:
            self._holding = [False, False]
            self._picked = []
        done(True, "")

    def set_suction(self, gripper, on, done):
        with self._lock:
            self._suction[gripper] = bool(on)
            if not on:  # like task_commander: releasing a holding gripper's suction drops what it held
                self._holding[gripper] = False
        done(True, "")

    def lift_to(self, gripper, height, done):
        with self._lock:
            lift = self._lift[gripper]
            prev, lift["done"] = lift["done"], done
            lift.update(goal=float(height), moving=True, arrived=False)
        if prev:
            prev(False, "superseded")

    def lift_cancel(self, gripper):
        with self._lock:
            lift = self._lift[gripper]
            done, lift["done"] = lift["done"], None
            lift.update(moving=False, arrived=False, goal=lift["h"])
        if done:
            done(False, "canceled")

    def lift_zero(self, gripper, done):
        with self._lock:
            prev, self._lift[gripper]["done"] = self._lift[gripper]["done"], None
            self._lift[gripper].update(h=0.0, goal=0.0, moving=False, arrived=True)
        if prev:
            prev(False, "zeroed")
        done(True, "")

    def snapshot(self):
        with self._lock:
            fire = self._advance()
            snap = Snapshot(
                pose=tuple(self._pose), vel=self._cmd,
                wheels=motor_snapshot(self._cmd, self._suction),
                grippers=[(l["h"], l["goal"], l["moving"], l["arrived"], self._suction[i], True)
                          for i, l in enumerate(self._lift)],
                scan=self._scan(), path=list(self._path), task_text=self._task_text(),
                tracker_enabled=self._tracker)
        self._run(fire)  # outside our lock: callbacks take the supervisor lock
        return snap

    def path_names(self):
        return list(PATHS)


def main(argv=None):
    ap = argparse.ArgumentParser(description="BridgeServer on a simulated robot (no ROS)")
    ap.add_argument("--port", type=int, default=P.PORT)
    ap.add_argument("--beacon-port", type=int, default=P.BEACON_PORT)
    ap.add_argument("--beacon-addr", default="255.255.255.255")
    args = ap.parse_args(argv)
    from .server import BridgeServer  # local: keeps the module import light

    config = expand_env(DEFAULT_CONFIG)
    server = BridgeServer(config["robot_name"], FakeRosIo(config), config,
                          log_root=Path.home() / ".oumuamua_controller" / "console",
                          port=args.port, beacon_port=args.beacon_port,
                          beacon_addr=args.beacon_addr)
    server.start()
    print(f"fake robot '{config['robot_name']}' listening on UDP {server.port} (Ctrl-C to stop)")
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()


if __name__ == "__main__":
    main()
