import random
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from oumuamua_controller_bridge.reliable import ReliableReceiver, ReliableSender  # noqa: E402


def test_in_order_delivery():
    r = ReliableReceiver()
    assert r.on_rel(1, ["a"]) == [["a"]]
    assert r.on_rel(2, ["b"]) == [["b"]]
    assert r.cum == 2


def test_out_of_order_buffered():
    r = ReliableReceiver()
    assert r.on_rel(3, ["c"]) == []
    assert r.on_rel(2, ["b"]) == []
    assert r.cum == 0
    assert r.on_rel(1, ["a"]) == [["a"], ["b"], ["c"]]
    assert r.cum == 3


def test_duplicates_dropped():
    r = ReliableReceiver()
    assert r.on_rel(1, ["a"]) == [["a"]]
    assert r.on_rel(1, ["a"]) == []
    assert r.on_rel(3, ["c"]) == []
    assert r.on_rel(3, ["c"]) == []
    assert r.on_rel(2, ["b"]) == [["b"], ["c"]]


def test_beyond_window_dropped():
    r = ReliableReceiver()
    assert r.on_rel(65, ["x"]) == []
    assert r.on_rel(64, ["y"]) == []
    for i in range(1, 63):
        assert r.on_rel(i, [i]) == [[i]]
    # 64 was buffered; 63 releases 63 and 64. 65 was dropped.
    assert r.on_rel(63, [63]) == [[63], ["y"]]
    assert r.cum == 64
    assert r.on_rel(65, ["x"]) == [["x"]]


def test_retransmit_after_rto():
    s = ReliableSender()
    s.push(["a"])
    assert s.poll(0) == [(1, ["a"])]
    assert s.poll(99) == []
    assert s.poll(100) == [(1, ["a"])]
    assert s.poll(199) == []
    assert s.poll(200) == [(1, ["a"])]
    s.on_ack(1)
    assert s.poll(400) == []
    assert s.pending() == 0


def test_window_limits_inflight():
    s = ReliableSender()
    for i in range(100):
        s.push([i])
    out = s.poll(0)
    assert [q for q, _ in out] == list(range(1, 65))
    assert s.pending() == 100
    assert s.poll(1) == []
    s.on_ack(10)
    assert s.pending() == 90
    out = s.poll(2)
    assert [q for q, _ in out] == list(range(65, 75))


def test_stale_ack_ignored_and_reset():
    s = ReliableSender()
    s.push([1])
    s.push([2])
    s.poll(0)
    s.on_ack(2)
    s.on_ack(1)
    assert s.pending() == 0
    s.on_ack(99)
    assert s.pending() == 0
    s.push([3])
    assert s.poll(5) == [(3, [3])]
    s.reset()
    assert s.pending() == 0
    s.push(["n"])
    assert s.poll(6) == [(1, ["n"])]
    r = ReliableReceiver()
    r.on_rel(1, ["a"])
    r.on_rel(3, ["c"])
    r.reset()
    assert r.cum == 0
    assert r.on_rel(1, ["z"]) == [["z"]]
    assert r.on_rel(2, ["y"]) == [["y"]]


def test_lossy_link_simulation():
    rng = random.Random(12345)
    n = 500
    s = ReliableSender()
    r = ReliableReceiver()
    for i in range(n):
        s.push([i])
    got = []
    to_recv = []  # (seq,msg) in flight to receiver
    to_send = []  # acks in flight to sender

    def transmit(q, item):
        if rng.random() < 0.3:
            return
        q.append(item)
        if rng.random() < 0.2:
            q.append(item)
        if len(q) > 1 and rng.random() < 0.3:
            q[-1], q[-2] = q[-2], q[-1]

    now = 0
    while len(got) < n and now < 200000:
        for seq, msg in s.poll(now):
            transmit(to_recv, (seq, msg))
        batch, to_recv[:] = list(to_recv), []
        for seq, msg in batch:
            got.extend(r.on_rel(seq, msg))
            transmit(to_send, r.cum)
        batch, to_send[:] = list(to_send), []
        for c in batch:
            s.on_ack(c)
        now += 10
    assert got == [[i] for i in range(n)]
    assert s.pending() == 0 or len(got) == n
