import socket
import time

import pytest

from fake_rosio import FakeRosIo
from oumuamua_controller_bridge import protocol as P
from oumuamua_controller_bridge.rosio import Snapshot
from oumuamua_controller_bridge.server import BridgeServer

CONFIG = {
    "robot_name": "oum", "lift_presets": [["下段", 0.01], ["上段", 0.69]], "lift_max": 0.19,
    "speed_levels": [[0.3, 1.0], [0.8, 1.8], [1.5, 2.5]], "blocks": 6, "columns": 3, "rows": 3,
    "field_bounds": [-2.786, -1.655, 2.786, 1.655],
    "field_obstacles": [-0.8375, 0.019, -0.8045, 0.8905],
    "shelf": [1.5625, -0.30, 2.5025, 0.30], "start_pose": [-2.2, 1.3, 0.0],
    "robot_radius": 0.32, "completion_setup_files": [],
}


class Phone:
    def __init__(self, port, name="phone", session=1234):
        self.port = port
        self.name = name
        self.session = session
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.05)
        self.rel_seq = 0
        self.req = 0
        self.rx = []  # decoded datagrams
        self.rel_msgs = []
        self.cum = 0
        self.ctl_seq = 0

    def send(self, v):
        self.sock.sendto(P.encode(v), ("127.0.0.1", self.port))

    def hello(self, takeover=False):
        self.send([P.HELLO, self.session, self.name, takeover, None])

    def control(self, **kw):
        self.ctl_seq += 1
        c = dict(vx=0.0, vy=0.0, wz=0.0, field=False, jl=0, jr=0, estop=False)
        c.update(kw)
        self.send([P.CONTROL, self.session, self.ctl_seq, self.ctl_seq * 10, c["vx"], c["vy"],
                   c["wz"], c["field"], c["jl"], c["jr"], c["estop"]])

    def rel(self, msg):
        self.rel_seq += 1
        self.send([P.REL, self.session, self.rel_seq, msg])

    def request(self, typ, *args):
        self.req += 1
        self.rel([typ, self.req, *args])
        return self.req

    def pump(self, secs):
        end = time.monotonic() + secs
        while time.monotonic() < end:
            try:
                data, _ = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            m = P.decode(data)
            if m is None:
                continue
            self.rx.append(m)
            if m[0] == P.REL and m[1] == self.session:
                if m[2] == self.cum + 1:
                    self.cum = m[2]
                    self.rel_msgs.append(m[3])
                self.send([P.ACK, self.session, self.cum])

    def wait(self, pred, secs=2.0):
        end = time.monotonic() + secs
        while time.monotonic() < end:
            self.pump(0.05)
            hit = pred()
            if hit:
                return hit
        return None

    def kinds(self, k):
        return [m for m in self.rx if m[0] == k]

    def rels(self, t):
        return [m for m in self.rel_msgs if m[0] == t]

    def connect(self, takeover=False):
        self.hello(takeover)
        w = self.wait(lambda: self.kinds(P.WELCOME))
        return w[-1] if w else None

    def close(self):
        self.sock.close()


class BrokenSnapshotIo(FakeRosIo):
    def snapshot(self):
        raise RuntimeError("boom")


@pytest.fixture
def env(tmp_path, request):
    io = getattr(request, "param", FakeRosIo)()
    srv = BridgeServer("oum", io, dict(CONFIG), tmp_path, port=0, beacon_port=0,
                       beacon_addr="127.0.0.1",
                       operator_timeout_s=getattr(request.module, "OP_TIMEOUT", 5.0))
    srv.start()
    phones = []

    def mk(**kw):
        p = Phone(srv.port, **kw)
        phones.append(p)
        return p

    yield io, srv, mk
    srv.stop()
    for p in phones:
        p.close()


def test_hello_welcome_and_initial_burst(env):
    io, srv, mk = env
    io.snap = Snapshot(task_text="holding: g0=yes g1=no\npicked blocks: 1 2\n")
    io.names = ["slalom", "start"]
    p = mk()
    w = p.connect()
    assert w[:5] == [P.WELCOME, 1234, "oum", True, ""]
    p.wait(lambda: len(p.rels(P.TASK_MEMORY)) >= 1)
    types = [m[0] for m in p.rel_msgs]
    assert types[:4] == [P.CONFIG, P.FIELD, P.PATH_SHAPE, P.TASK_MEMORY]
    cfg = p.rels(P.CONFIG)[0]
    assert cfg[1] == "oum" and cfg[2] == ["slalom", "start"] and cfg[4] == 0.19
    field = p.rels(P.FIELD)[0]
    assert len(field[1]) == 4 and len(field[2]) == 1 and field[4] == [-2.2, 1.3, 0.0] and field[5] == 0.32
    mem = p.rels(P.TASK_MEMORY)[0]
    assert mem[1] == [True, False] and mem[2] == [1, 2]


def test_second_client_busy_then_takeover_sends_bye(env):
    io, srv, mk = env
    a = mk(name="a", session=11)
    assert a.connect()[3] is True
    b = mk(name="b", session=22)
    w = b.connect()
    assert w[3] is False and w[4] == "busy"
    a.pump(0.2)
    assert not a.kinds(P.BYE)
    w = b.connect(takeover=True)
    assert w[3] is True
    assert a.wait(lambda: a.kinds(P.BYE))[-1][2] == "taken over"
    # old session's CONTROL is ignored
    a.control(vx=1.0)
    b.control(vx=0.5)
    b.pump(0.15)
    assert any(c[0] == "cmd_vel" for c in io.calls)


def test_same_client_name_reconnects_without_busy_or_bye(env):
    io, srv, mk = env
    a = mk(name="same", session=11)
    a.connect()
    a.wait(lambda: a.rels(P.TASK_MEMORY))
    a.session = 12
    a.rel_seq = 0
    a.cum = 0
    a.rel_msgs.clear()
    w = a.connect()
    assert w[3] is True and w[1] == 12
    assert a.wait(lambda: a.rels(P.CONFIG))
    assert not a.kinds(P.BYE)


def test_control_timeout_stops(env):
    io, srv, mk = env
    p = mk()
    p.connect()
    p.control()
    p.request(P.RELEASE_STOP)
    p.pump(0.1)
    for _ in range(5):
        p.control(vx=0.5)
        p.pump(0.02)
    p.pump(0.05)
    assert srv.sup.mode == 0
    p.pump(0.6)
    assert srv.sup.mode == 2 and srv.sup.stop_reason == "link lost"


def test_state_stream_has_ack(env):
    io, srv, mk = env
    io.snap = Snapshot(pose=(1.0, 2.0, 0.5), wheels=[(1.5, True)])
    p = mk()
    p.connect()
    p.control()
    p.control()
    st = p.wait(lambda: [m for m in p.kinds(P.STATE) if m[4] == 20])
    assert st
    m = st[-1]
    assert m[1] == 1234 and m[3] == 2
    body = m[5]
    assert len(body) == 9
    assert body[0] in (0, 1, 2) and body[2] == [1.0, 2.0, 0.5] and body[8] is False
    seqs = [x[2] for x in p.kinds(P.STATE)]
    assert seqs == sorted(seqs)


def test_scan_fragments_filter_nonfinite(env):
    io, srv, mk = env
    io.snap = Snapshot(scan=[(float("nan"), 0.0), (1.0, 2.0)] + [(0.1, 0.1)] * 300)
    p = mk()
    p.connect()
    got = p.wait(lambda: p.kinds(P.SCAN))
    first = got[0]
    sid = first[2]
    frags = {m[3]: m for m in p.kinds(P.SCAN) if m[2] == sid}
    assert first[4] == 2
    pts = P.unpack_points(frags[0][5])
    assert len(pts) == 280 and pts[0] == (1.0, 2.0)


def test_reliable_shell_roundtrip(env):
    io, srv, mk = env
    p = mk()
    p.connect()
    p.request(P.SHELL_OPEN, "t1")
    assert p.wait(lambda: p.rels(P.SHELL_OPENED))
    assert p.wait(lambda: [r for r in p.rels(P.RESULT) if r[1] == 1])[-1][2] is True
    p.request(P.SHELL_INPUT, "t1", b"echo hi\n")
    assert p.wait(lambda: b"hi" in b"".join(
        m[2] for m in p.rels(P.SHELL_OUTPUT)))
    p.request(P.SHELL_CLOSE, "t1")
    closed = p.wait(lambda: p.rels(P.SHELL_CLOSED))
    assert closed[-1][1:] == ["t1", "closed"] and len(closed) == 1


def test_shell_exit_sends_closed_exited(env):
    io, srv, mk = env
    p = mk()
    p.connect()
    p.request(P.SHELL_OPEN, "t1")
    p.wait(lambda: p.rels(P.SHELL_OPENED))
    p.request(P.SHELL_INPUT, "t1", b"exit\n")
    closed = p.wait(lambda: p.rels(P.SHELL_CLOSED))
    assert closed[-1][1:] == ["t1", "exited"]


def test_unknown_rel_kind_is_acked_and_dropped(env):
    io, srv, mk = env
    p = mk()
    p.connect()
    p.rel([99, 1])
    ack = p.wait(lambda: [m for m in p.kinds(P.ACK) if m[2] >= 1])
    assert ack
    assert not p.rels(P.RESULT)


def test_log_filtered_by_level_and_notify(env):
    io, srv, mk = env
    p = mk()
    p.connect()
    p.request(P.LOG_LEVEL, 30)
    p.wait(lambda: p.rels(P.RESULT))
    srv.notify([P.LOG, 1, 20, "n", "info"])
    srv.notify([P.LOG, 2, 40, "n", "error"])
    assert p.wait(lambda: p.rels(P.LOG))
    p.pump(0.2)
    assert [m[4] for m in p.rels(P.LOG)] == ["error"]


def test_beacon_sent(tmp_path):
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0))
    rx.settimeout(2.0)
    srv = BridgeServer("oum", FakeRosIo(), dict(CONFIG), tmp_path, port=0,
                       beacon_port=rx.getsockname()[1], beacon_addr="127.0.0.1")
    srv.start()
    try:
        data, _ = rx.recvfrom(1024)
    finally:
        srv.stop()
        rx.close()
    m = P.decode(data)
    assert m == [P.BEACON, "oum", srv.port]


def test_output_flood_does_not_delay_state(env):
    io, srv, mk = env
    p = mk()
    p.connect()
    p.request(P.SHELL_OPEN, "t1")
    p.wait(lambda: p.rels(P.SHELL_OPENED))
    p.pump(0.2)
    p.request(P.SHELL_INPUT, "t1", b"yes | head -c 5000000\n")
    p.pump(0.2)
    n0 = len(p.kinds(P.STATE))
    t0 = time.monotonic()
    p.pump(1.5)
    dt = time.monotonic() - t0
    got = len(p.kinds(P.STATE)) - n0
    assert got >= 0.8 * 20 * dt, (got, dt)


@pytest.mark.parametrize("env", [BrokenSnapshotIo], indirect=True)
def test_deadman_survives_snapshot_failure(env):
    io, srv, mk = env
    p = mk()
    p.connect()
    p.control()
    p.request(P.RELEASE_STOP)
    p.pump(0.1)
    assert srv.sup.mode == 0
    n = len(io.calls)
    p.pump(0.6)
    assert srv.sup.mode == 2
    assert sum(1 for c in io.calls[n:] if c == ("cmd_vel", 0.0, 0.0, 0.0)) >= 5


def test_stale_operator_expires(env):
    io, srv, mk = env
    srv._operator_timeout = 0.3
    a = mk(name="a", session=11)
    a.connect()
    b = mk(name="b", session=22)
    assert b.connect()[3] is False  # still fresh
    a.pump(0.5)
    w = b.connect()
    assert w[3] is True and w[1] == 22
    assert not a.kinds(P.BYE)


def test_repeated_hello_updates_address(env):
    io, srv, mk = env
    a = mk(name="a", session=11)
    a.connect()
    a2 = mk(name="a", session=11)  # same session from a new source port
    a2.hello()
    assert a2.wait(lambda: a2.kinds(P.WELCOME))
    assert a2.wait(lambda: a2.kinds(P.STATE))  # streams now go to the new address


def test_oversize_messages_are_truncated(env):
    io, srv, mk = env
    p = mk()
    p.connect()
    srv.notify([P.LOG, 1, 50, "n", "あ" * 5000])
    srv.handlers.send([P.SHELL_COMPLETION, 1, "t", 0, ["cand%04d" % i for i in range(2000)]])
    srv.handlers.send([P.RESULT, 2, False, "x" * 5000])
    assert p.wait(lambda: p.rels(P.SHELL_COMPLETION) and p.rels(P.LOG) and p.rels(P.RESULT))
    assert 0 < len(p.rels(P.SHELL_COMPLETION)[0][4]) < 2000
    assert all(len(P.encode(m)) <= 1200 for m in p.rx)


def test_decimate_keeps_ends_and_limit():
    from oumuamua_controller_bridge.server import decimate

    pts = [(i * 0.01, 0.0) for i in range(483)]
    out = decimate(pts, 280)
    assert len(out) == 280 and out[0] == pts[0] and out[-1] == pts[-1]
    assert decimate(pts[:10], 280) == pts[:10]


def test_default_field_and_long_path_fit_one_datagram():
    from oumuamua_controller_bridge import fake_robot as F
    from oumuamua_controller_bridge.server import (
        MAX_DATAGRAM, _box, _obstacle_boxes, decimate)

    c = F.expand_env(F.DEFAULT_CONFIG)
    field = [P.FIELD, _box(c["field_bounds"]), _obstacle_boxes(c.get("field_obstacles")),
             _box(c["shelf"]), list(c["start_pose"]), float(c["robot_radius"])]
    assert len(P.encode([P.REL, 0xFFFFFFFF, 0xFFFFFFFF, field])) <= MAX_DATAGRAM
    pts = decimate([(i * 0.01, 0.0) for i in range(483)], P.MAX_POINTS_PER_SCAN)
    shape = [P.PATH_SHAPE, "long_path_name", P.pack_points(pts)[0]]
    assert len(P.encode([P.REL, 0xFFFFFFFF, 0xFFFFFFFF, shape])) <= MAX_DATAGRAM
    assert P.unpack_points(shape[2])[-1][0] == 4.82
