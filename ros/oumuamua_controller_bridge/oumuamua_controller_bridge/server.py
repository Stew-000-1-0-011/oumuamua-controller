"""UDP bridge server: one worker thread runs receive, supervisor tick, STATE/SCAN/BEACON and
shell polling (see docs/protocol.md). No rclpy here."""
import math
import queue
import socket
import sys
import threading
import time
import traceback
from pathlib import Path
from typing import Optional

from . import protocol as P
from . import task_memory
from .completion import complete as _complete
from .handlers import Handlers
from .reliable import ReliableReceiver, ReliableSender
from .rosio import RosIo, Snapshot
from .shells import ShellManager
from .supervisor import Supervisor

TICK_S = 0.02
STATE_S = 0.05
SCAN_S = 0.1
BEACON_S = 1.0
FLOOD_PENDING = 64
RESYNC_PENDING = 16
LOG_PENDING = 32
RESYNC_BYTES = 64 * 1024
MAX_PATH_POINTS = P.MAX_POINTS_PER_SCAN


def _finite_points(points) -> list:
    out = []
    for p in points or []:
        try:
            x, y = float(p[0]), float(p[1])
        except (TypeError, ValueError, IndexError):
            continue
        if math.isfinite(x) and math.isfinite(y):
            out.append((x, y))
    return out


MAX_DATAGRAM = 1200
_REL_OVERHEAD = [P.REL, 2**32 - 1, 2**32 - 1]


def _cut(text, nbytes: int) -> str:
    return str(text).encode("utf-8")[:max(0, nbytes)].decode("utf-8", "ignore")


def _fits(msg: list) -> bool:
    return len(P.encode(_REL_OVERHEAD + [msg])) <= MAX_DATAGRAM


def _fit(msg: list) -> list:
    """Shrink LOG text, RESULT message and SHELL_COMPLETION candidates to fit one datagram."""
    if not msg or _fits(msg):
        return msg
    msg = list(msg)
    try:
        if msg[0] == P.LOG:
            msg[3] = _cut(msg[3], 64)
            msg[4] = _cut(msg[4], 700)
        elif msg[0] == P.RESULT:
            msg[3] = _cut(msg[3], 900)
        elif msg[0] == P.SHELL_COMPLETION:
            msg[2] = _cut(msg[2], 64)
            cands = [_cut(c, 200) for c in msg[4]]
            while cands and not _fits(msg[:4] + [cands]):
                cands.pop()
            msg[4] = cands
    except (IndexError, TypeError):
        pass
    return msg


def _box(b) -> list:
    return [float(v) for v in b]


def _obstacle_boxes(obstacles) -> list:
    """Config obstacles: a list of boxes or a flat list of 4n numbers -> [[xmin, ymin, xmax, ymax]]."""
    flat = list(obstacles or [])
    if flat and isinstance(flat[0], (list, tuple)):
        return [_box(b) for b in flat if len(b) == 4]
    return [_box(flat[i:i + 4]) for i in range(0, len(flat) - 3, 4)]


def decimate(pts: list, limit: int) -> list:
    """At most `limit` points, uniform stride over the whole path, first and last kept."""
    n = len(pts)
    if n <= limit:
        return pts
    return [pts[round(i * (n - 1) / (limit - 1))] for i in range(limit)]


class BridgeServer:
    """Tracker flag in STATE: read from `Snapshot.tracker_enabled` when the RosIo provides it,
    otherwise False (the field is optional on Snapshot)."""

    def __init__(self, robot_name: str, io: RosIo, config: dict, log_root: Path,
                 port: int = P.PORT, beacon_port: int = P.BEACON_PORT,
                 beacon_addr: str = "255.255.255.255", operator_timeout_s: float = 5.0):
        self.robot_name = robot_name
        self.io = io
        self.config = dict(config)
        self._beacon_to = (beacon_addr, beacon_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.004)
        self.port = self.sock.getsockname()[1]

        self._txlock = threading.Lock()
        self._sender = ReliableSender()
        self._receiver = ReliableReceiver()
        self._notify_q: "queue.Queue[list]" = queue.Queue()
        self._thread: Optional[threading.Thread] = None
        self._running = False

        self._session: Optional[int] = None
        self._client_name = ""
        self._addr = None
        self._operator_timeout = operator_timeout_s
        self._last_op_rx = 0.0
        self._max_ctl_seq = 0
        self._ack_seq = 0
        self._ack_t = 0
        self._state_seq = 0
        self._scan_id = 0
        self._last_task_text = object()
        self._path_shape: Optional[list] = None
        self._dropped_tabs: set = set()  # shell output was dropped: resync when the link drains

        self.sup = Supervisor(io, lift_max=float(config.get("lift_max", 0.19)))
        self.shells = ShellManager(Path(log_root), self._on_shell_output, self._on_shell_closed)
        setup = tuple(config.get("completion_setup_files", ()))
        self.handlers = Handlers(
            io, self.sup, self.shells,
            lambda line, cursor, cwd: _complete(line, cursor, cwd, setup),
            self._send, self.config)

    # -- public ---------------------------------------------------------
    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._run, name="bridge-server", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        try:
            self.shells.close_all()
        finally:
            self.sock.close()

    def notify(self, msg: list) -> None:
        """Thread-safe: queue a bridge->phone Msg (LOG, PATH_SHAPE, ...) for reliable delivery."""
        self._notify_q.put(msg)

    # -- sending --------------------------------------------------------
    def _send(self, msg: list) -> None:
        """Non-blocking: append to the reliable sender (dropped when nobody is connected)."""
        msg = _fit(msg)
        with self._txlock:
            if self._session is not None:
                self._sender.push(msg)

    def _tx(self, value: list, addr=None) -> None:
        addr = addr or self._addr
        if addr is None:
            return
        data = P.encode(value)
        if len(data) > MAX_DATAGRAM:
            print(f"warning: outgoing datagram of {len(data)} bytes exceeds {MAX_DATAGRAM} (kind {value[0] if value else '?'})",
                  file=sys.stderr)
        try:
            self.sock.sendto(data, addr)
        except OSError:
            pass

    def _on_shell_output(self, tab: str, data: bytes) -> None:
        for i in range(0, len(data), P.MAX_SHELL_CHUNK):
            with self._txlock:
                if self._session is None:
                    return
                if self._sender.pending() >= FLOOD_PENDING:
                    self._dropped_tabs.add(tab)  # stays in the replay buffer; resent later
                    return
                self._sender.push([P.SHELL_OUTPUT, tab, data[i:i + P.MAX_SHELL_CHUNK]])

    def _on_shell_closed(self, tab: str, reason: str) -> None:
        self._send([P.SHELL_CLOSED, tab, reason])

    # -- session --------------------------------------------------------
    def _handle_hello(self, m: list, addr, now: float) -> None:
        session, name, takeover = int(m[1]), str(m[2]), bool(m[3])
        if session == 0:
            return
        if self._session is not None and name != self._client_name and session != self._session \
                and now - self._last_op_rx > self._operator_timeout:
            self._release_operator()  # the old operator is gone: no busy, no BYE
        if self._session == session:
            self._addr = addr
            self._last_op_rx = now
            self._tx([P.WELCOME, session, self.robot_name, True, ""], addr)
            return
        if self._session is not None and name != self._client_name:
            if not takeover:
                self._tx([P.WELCOME, session, self.robot_name, False, "busy"], addr)
                return
            self._tx([P.BYE, self._session, "taken over"], self._addr)
        self._new_session(session, name, addr, now)

    def _release_operator(self) -> None:
        with self._txlock:
            self._session = None
            self._addr = None

    def _new_session(self, session: int, name: str, addr, now: float) -> None:
        self.sup.link_lost()  # protocol 5.1: stopped until CONTROL + release
        with self._txlock:
            self._session, self._client_name, self._addr = session, name, addr
            self._last_op_rx = now
            self._sender.reset()
            self._receiver.reset()
            self._dropped_tabs.clear()
            self._max_ctl_seq = 0
            self._ack_seq = self._ack_t = 0
        self._tx([P.WELCOME, session, self.robot_name, True, ""])
        self._initial_burst()

    def _initial_burst(self) -> None:
        c = self.config
        try:
            snap = self.io.snapshot()
        except Exception:
            traceback.print_exc()
            snap = Snapshot()
        burst = [
            [P.CONFIG, self.robot_name, list(self.io.path_names()),
             [list(p) for p in c.get("lift_presets", [])], float(c.get("lift_max", 0.19)),
             [list(s) for s in c.get("speed_levels", [])], int(c.get("blocks", 6)),
             int(c.get("columns", 3)), int(c.get("rows", 3))],
            [P.FIELD, _box(c.get("field_bounds", [0, 0, 0, 0])),
             _obstacle_boxes(c.get("field_obstacles")),
             _box(c.get("shelf", [0, 0, 0, 0])), list(c.get("start_pose", [0, 0, 0])),
             float(c.get("robot_radius", 0.32))],
            self._path_shape_msg(snap),
            self._task_memory_msg(snap.task_text),
        ]
        self._last_task_text = snap.task_text
        for tab in self.shells.tabs():
            replay = self.shells.replay(tab)
            # A REL must fit one datagram: OPENED carries the first chunk, OUTPUT the rest.
            burst.extend(P.shell_opened_msgs(tab, replay))
        with self._txlock:
            for msg in burst:
                self._sender.push(msg)

    def _path_shape_msg(self, snap) -> list:
        if self._path_shape is not None:
            return self._path_shape
        pts = _finite_points(snap.path)
        pts = decimate(pts, MAX_PATH_POINTS)
        cur = self.handlers.current_path
        name = cur[0] if cur and pts else ""
        return [P.PATH_SHAPE, name, P.pack_points(pts)[0]]

    @staticmethod
    def _task_memory_msg(text) -> list:
        holding, picked, placed = task_memory.parse(text or "")
        return [P.TASK_MEMORY, list(holding), list(picked), [list(c) for c in placed]]

    # -- receive --------------------------------------------------------
    def _on_datagram(self, data: bytes, addr, now: float) -> None:
        m = P.decode(data)
        if m is None:
            return
        try:
            kind = m[0]
            if kind == P.HELLO:
                self._handle_hello(m, addr, now)
            elif self._session is None or len(m) < 2 or m[1] != self._session:
                return
            elif self._note_rx(now):
                pass
            elif kind == P.CONTROL:
                self._on_control(m, now)
            elif kind == P.REL:
                self._on_rel(m)
            elif kind == P.ACK:
                with self._txlock:
                    self._sender.on_ack(int(m[2]))
            elif kind == P.BYE:
                self._release_operator()
                self.sup.link_lost()
        except (IndexError, TypeError, ValueError):
            pass
        except Exception:
            traceback.print_exc()

    def _note_rx(self, now: float) -> bool:
        self._last_op_rx = now
        return False

    def _on_control(self, m: list, now: float) -> None:
        seq = int(m[2])
        if seq <= self._max_ctl_seq:
            return
        self._max_ctl_seq = seq
        self._ack_seq, self._ack_t = seq, int(m[3])
        self.sup.on_control(now, dict(
            session=m[1], seq=seq, t_ms=m[3], vx=m[4], vy=m[5], wz=m[6], field_frame=m[7],
            jog_left=m[8], jog_right=m[9], estop=m[10]))

    def _on_rel(self, m: list) -> None:
        seq, msg = int(m[2]), m[3]
        delivered = self._receiver.on_rel(seq, msg) if isinstance(msg, list) else []
        self._tx([P.ACK, self._session, self._receiver.cum])
        for d in delivered:
            if isinstance(d, list) and d:
                self.handlers.handle(d)  # unknown kinds are ignored by Handlers (still ACKed)

    # -- periodic -------------------------------------------------------
    def _send_state(self, snap) -> None:
        with self.sup.lock:
            mode, reason = int(self.sup.mode), self.sup.stop_reason
            task, path = self.handlers.current_task, self.handlers.current_path
        body = [
            mode, reason,
            [float(v) for v in snap.pose] if snap.pose else None,
            [float(v) for v in snap.vel] if snap.vel else None,
            [[float(w[0]), bool(w[1])] for w in snap.wheels],
            [self._gripper(g) for g in list(snap.grippers)[:2]],
            [int(task[0]), str(task[1]), [int(a) for a in task[2]]] if task else None,
            [str(path[0]), bool(path[1])] if path else None,
            bool(getattr(snap, "tracker_enabled", False)),
        ]
        self._state_seq += 1
        self._tx([P.STATE, self._session, self._state_seq, self._ack_seq, self._ack_t, body])
        if snap.task_text != self._last_task_text:
            self._last_task_text = snap.task_text
            self._send(self._task_memory_msg(snap.task_text))

    @staticmethod
    def _gripper(g):
        if g is None:
            return None
        h, goal, moving, arrived, suction, fb = list(g)[:6]
        return [float(h), float(goal), bool(moving), bool(arrived), bool(suction), bool(fb)]

    def _send_scan(self, snap) -> None:
        chunks = P.pack_points(_finite_points(snap.scan))
        self._scan_id += 1
        for i, ch in enumerate(chunks):
            self._tx([P.SCAN, self._session, self._scan_id, i, len(chunks), ch])

    def _drain_notify(self) -> None:
        while True:
            try:
                msg = self._notify_q.get_nowait()
            except queue.Empty:
                return
            if msg and msg[0] == P.LOG:
                if len(msg) > 2 and msg[2] < self.handlers.log_level:
                    continue
                with self._txlock:
                    if self._sender.pending() > LOG_PENDING:
                        continue  # a slow link must not pile up logs
            if msg and msg[0] == P.PATH_SHAPE:
                self._path_shape = msg
            self._send(msg)

    def _resync_shells(self) -> None:
        """Output was dropped under load: resend the tab's replay once the queue has drained."""
        with self._txlock:
            if not self._dropped_tabs or self._session is None \
                    or self._sender.pending() >= RESYNC_PENDING:
                return
            tabs, self._dropped_tabs = self._dropped_tabs, set()
        for tab in tabs:
            if tab not in self.shells.tabs():
                continue
            replay = self.shells.replay(tab)
            if len(replay) > RESYNC_BYTES:
                replay = replay[-RESYNC_BYTES:]
                nl = replay.find(b"\n")
                replay = replay[nl + 1:] if nl >= 0 else replay
            with self._txlock:
                for msg in P.shell_opened_msgs(tab, replay):
                    self._sender.push(msg)

    def _check_operator(self, now: float) -> None:
        """A silent operator (5 s without any datagram) is released and the robot stopped."""
        if self._session is not None and now - self._last_op_rx > self._operator_timeout:
            self._release_operator()
            self.sup.link_lost()

    def _flush_reliable(self, now: float) -> None:
        with self._txlock:
            if self._session is None:
                return
            items = self._sender.poll(int(now * 1000))
            session = self._session
        for seq, msg in items:
            self._tx([P.REL, session, seq, msg])

    def _run(self) -> None:
        t0 = time.monotonic()
        next_tick = next_state = next_scan = next_beacon = t0
        while self._running:
            try:
                data, addr = self.sock.recvfrom(4096)
                self._on_datagram(data, addr, time.monotonic())
                for _ in range(200):  # drain the backlog without waiting
                    self.sock.settimeout(0)
                    try:
                        data, addr = self.sock.recvfrom(4096)
                    except (BlockingIOError, socket.timeout):
                        break
                    self._on_datagram(data, addr, time.monotonic())
            except (socket.timeout, BlockingIOError):
                pass
            except OSError:
                if not self._running:
                    break
            finally:
                try:
                    self.sock.settimeout(0.004)
                except OSError:
                    break
            now = time.monotonic()
            cache = {}

            def snap():
                """One io.snapshot() per loop turn, taken lazily; None if it raises."""
                if "s" not in cache:
                    try:
                        cache["s"] = self.io.snapshot()
                    except Exception:
                        traceback.print_exc()
                        cache["s"] = None
                return cache["s"]

            def guarded(fn):
                try:
                    fn()
                except Exception:
                    traceback.print_exc()

            if now >= next_tick:  # first: the deadman must not depend on anything else
                next_tick = max(next_tick + TICK_S, now - TICK_S)

                def tick():
                    s = snap()
                    yaw = s.pose[2] if s is not None and s.pose else None
                    self.sup.tick(now, yaw)
                guarded(tick)
            guarded(lambda: self._check_operator(now))
            guarded(lambda: self.shells.poll(0))
            guarded(self._resync_shells)
            guarded(self._drain_notify)
            if now >= next_state:
                next_state = max(next_state + STATE_S, now - STATE_S)
                if self._session is not None and snap() is not None:
                    guarded(lambda: self._send_state(snap()))
            if now >= next_scan:
                next_scan = max(next_scan + SCAN_S, now - SCAN_S)
                if self._session is not None and snap() is not None:
                    guarded(lambda: self._send_scan(snap()))
            if now >= next_beacon:
                next_beacon = now + BEACON_S
                guarded(lambda: self._tx([P.BEACON, self.robot_name, self.port], self._beacon_to))
            guarded(lambda: self._flush_reliable(now))
