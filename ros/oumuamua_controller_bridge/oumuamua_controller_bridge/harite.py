"""Harite (a lift-only mechanism without suction): pure logic, no rclpy.

The robot side (harite_node) takes `/harite/target_velocity` [rotor rad/s] and needs it published
continuously (after 0.2 s of silence it sends 0), plus `/harite/enable` (SetBool). It has no
position, no presets and no limits, so the operator stops at the ends of travel.

The phone's CONTROL carries `jog_harite` (-1, 0, +1; +1 is "up" as labelled). While the supervisor
is MANUAL, every tick publishes `jog * speed` (0 when released), exactly like the wheels: if CONTROL
stops, the supervisor stops and so does the stream, and the node's own timeout covers the rest.
"""
import math
import sys
import traceback
from typing import Optional

DEFAULT_SPEED = 30.0   # rad/s
MAX_SPEED = 100.0      # harite.yaml max_velocity: the node clamps there anyway
RETRY_S = 1.0          # re-send /harite/enable while the node has not confirmed the wanted state


def clamp_speed(v) -> float:
    """The `harite_speed` parameter clamped to 0..MAX_SPEED. Not a finite number: the default."""
    try:
        v = float(v)
    except (TypeError, ValueError):
        return DEFAULT_SPEED
    if not math.isfinite(v):
        return DEFAULT_SPEED
    return max(0.0, min(MAX_SPEED, v))


def jog_sign(v) -> int:
    """CONTROL's jog_harite as -1, 0 or +1. Absent, nil, NaN or not a number is 0 (an older phone)."""
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return 0
    if not math.isfinite(v) or v == 0:
        return 0
    return 1 if v > 0 else -1


def target_velocity(jog: int, speed: float, reverse: bool) -> float:
    """Rotor rad/s for a jog direction. `reverse` flips the sign when "up" is negative on the mount."""
    v = float(jog_sign(jog)) * clamp_speed(speed)
    if reverse:
        v = -v
    return v + 0.0  # never -0.0


def command_value(v) -> Optional[float]:
    """A /harite/command value for STATE: None (nil on the wire) when unknown or not finite."""
    if v is None:
        return None
    try:
        v = float(v)
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


class HariteDriver:
    """Used by the Supervisor, always with the supervisor's lock held (`lock` is the same RLock; the
    `/harite/enable` reply may arrive on another thread and takes it).

    Rules:
      * MANUAL: each tick publishes the held direction (0 when released).
      * STOP (estop, CONTROL loss, latch): publish 0 at once, then call /harite/enable false. Until the
        node confirms, 0 keeps being published and the call is repeated every RETRY_S, so a missing
        node or a lost reply cannot leave it enabled. Once disabled it is left alone.
      * RELEASE_STOP: call /harite/enable true (also repeated until confirmed). A button that was held
        through the stop (or through AUTO) does nothing until it has been let go.
      * AUTO: nothing is published (one 0 when auto begins); the node's 0.2 s timeout stops it.
    """

    def __init__(self, io, lock, speed: float, reverse: bool):
        self.io = io
        self.lock = lock
        self.speed = clamp_speed(speed)
        self.reverse = bool(reverse)
        self._jog = 0
        self._blocked = False   # held through a stop / auto: ignored until released
        self._want = False      # wanted /harite/enable state (the supervisor starts stopped)
        self._ok: Optional[bool] = None  # enable state the node confirmed (None: unknown)
        self._token = 0
        self._inflight = False
        self._sent_at = -math.inf

    # -- inputs ---------------------------------------------------------
    def on_control(self, jog, manual: bool) -> None:
        jog = jog_sign(jog)
        if manual:
            if jog == 0:
                self._blocked = False
        else:
            self._blocked = jog != 0
        self._jog = jog

    def stop(self, now: float) -> None:
        """Entering STOP: 0 first, then disable (so it cannot be left moving)."""
        self._jog = 0
        self._publish(0.0)
        self._set_want(False, now)

    def release(self, now: float) -> None:
        self._blocked = self._jog != 0
        self._set_want(True, now)

    def auto(self) -> None:
        self._publish(0.0)

    def tick(self, now: float, manual: bool, stopped: bool) -> None:
        if stopped:
            self._want = False
            if self._ok is not False:
                self._publish(0.0)
        elif not self._want:
            self._want = True
        if manual:
            self._publish(0.0 if self._blocked else target_velocity(self._jog, self.speed, self.reverse))
        self._reconcile(now, force=False)

    # -- internals ------------------------------------------------------
    def _publish(self, v: float) -> None:
        try:
            self.io.publish_harite(v)
        except Exception:
            print("harite: publish failed", file=sys.stderr)
            traceback.print_exc()

    def _set_want(self, on: bool, now: float) -> None:
        self._want = on
        self._reconcile(now, force=True)

    def _reconcile(self, now: float, force: bool) -> None:
        if self._ok == self._want:
            return
        if not force and (self._inflight or now - self._sent_at < RETRY_S):
            return
        self._token += 1
        token, want = self._token, self._want
        self._inflight = True
        self._sent_at = now
        try:
            self.io.set_harite_enabled(want, lambda ok, msg="": self._done(token, want, ok))
        except Exception:
            self._inflight = False
            print("harite: enable call failed", file=sys.stderr)
            traceback.print_exc()

    def _done(self, token: int, want: bool, ok: bool) -> None:
        with self.lock:
            if token != self._token:
                self._ok = None  # an older request landed after a newer one: say it again
                return
            self._inflight = False
            self._ok = want if ok else None
