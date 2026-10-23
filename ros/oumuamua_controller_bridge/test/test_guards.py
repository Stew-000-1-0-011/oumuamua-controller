from oumuamua_controller_bridge.guards import once, with_timeout


class FakeTimer:
    def __init__(self, seconds, fn):
        self.seconds, self.fn, self.cancelled, self.daemon = seconds, fn, False, False

    def start(self):
        pass

    def cancel(self):
        self.cancelled = True

    def fire(self):
        if not self.cancelled:
            self.fn()


def test_once_fires_once():
    calls = []
    d = once(lambda ok, m: calls.append((ok, m)))
    d(True, "a")
    d(False, "b")
    assert calls == [(True, "a")]


def test_once_swallows_exceptions():
    def boom(ok, m):
        raise RuntimeError("x")
    once(boom)(True, "")  # must not raise


def test_timeout_fires_false():
    calls, timers = [], []
    def factory(s, fn):
        timers.append(FakeTimer(s, fn))
        return timers[-1]
    d, _ = with_timeout(lambda ok, m: calls.append((ok, m)), "svc: 応答がありません", 5.0, factory)
    assert timers[0].seconds == 5.0
    timers[0].fire()
    d(True, "late")
    assert calls == [(False, "svc: 応答がありません")]


def test_reply_cancels_timeout():
    calls, timers = [], []
    def factory(s, fn):
        timers.append(FakeTimer(s, fn))
        return timers[-1]
    d, _ = with_timeout(lambda ok, m: calls.append((ok, m)), "t", 5.0, factory)
    d(True, "")
    timers[0].fire()
    assert calls == [(True, "")] and timers[0].cancelled
