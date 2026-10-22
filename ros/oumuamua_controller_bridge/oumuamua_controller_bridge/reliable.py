"""Reliable in-order channel (one direction). Pure, time-injected, no sockets."""


class ReliableSender:
    def __init__(self, window: int = 64, rto_ms: int = 100):
        self._window = window
        self._rto = rto_ms
        self.reset()

    def reset(self) -> None:
        self._acked = 0  # highest cumulatively acked seq
        self._next_seq = 1
        self._msgs: dict[int, list] = {}
        self._last_sent: dict[int, int] = {}

    def push(self, msg: list) -> None:
        self._msgs[self._next_seq] = msg
        self._next_seq += 1

    def poll(self, now_ms: int) -> list[tuple[int, list]]:
        out = []
        top = min(self._next_seq - 1, self._acked + self._window)
        for seq in range(self._acked + 1, top + 1):
            last = self._last_sent.get(seq)
            if last is None or now_ms - last >= self._rto:
                self._last_sent[seq] = now_ms
                out.append((seq, self._msgs[seq]))
        return out

    def on_ack(self, cum: int) -> None:
        cum = min(cum, self._next_seq - 1)
        while self._acked < cum:
            self._acked += 1
            self._msgs.pop(self._acked, None)
            self._last_sent.pop(self._acked, None)

    def pending(self) -> int:
        return self._next_seq - 1 - self._acked


class ReliableReceiver:
    def __init__(self, window: int = 64):
        self._window = window
        self.reset()

    def reset(self) -> None:
        self.cum = 0
        self._buf: dict[int, list] = {}

    def on_rel(self, seq: int, msg: list) -> list[list]:
        if seq <= self.cum or seq > self.cum + self._window:
            return []
        self._buf.setdefault(seq, msg)
        out = []
        while self.cum + 1 in self._buf:
            self.cum += 1
            out.append(self._buf.pop(self.cum))
        return out
