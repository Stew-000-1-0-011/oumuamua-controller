"""ROS-free helpers for `done` callbacks: fire exactly once, optionally with a timeout."""
import threading
import traceback

REPLY_TIMEOUT_S = 5.0


def once(done):
    """Wrap done so it runs at most once (thread-safe); exceptions from it are logged, not raised."""
    lock = threading.Lock()
    fired = [False]

    def guarded(ok, message=""):
        with lock:
            if fired[0]:
                return
            fired[0] = True
        try:
            done(ok, message)
        except Exception:
            traceback.print_exc()

    return guarded


class Watchdog:
    """done(False, message) after `seconds` unless cancelled first. `timer_factory` is injectable."""

    def __init__(self, seconds, fire, timer_factory=threading.Timer):
        self._timer = timer_factory(seconds, fire)
        self._timer.daemon = True
        self._timer.start()

    def cancel(self):
        self._timer.cancel()


def with_timeout(done, message, seconds=REPLY_TIMEOUT_S, timer_factory=threading.Timer):
    """Return (guarded_done, watchdog). guarded_done is once-only and stops the watchdog;
    on timeout done(False, message) is called instead."""
    guarded = once(done)
    dog = Watchdog(seconds, lambda: guarded(False, message), timer_factory)

    def finish(ok, msg=""):
        dog.cancel()
        guarded(ok, msg)

    return finish, dog
