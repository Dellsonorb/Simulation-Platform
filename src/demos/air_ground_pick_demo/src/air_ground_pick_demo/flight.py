"""Pure guards for the demo's single Prometheus flight cycle."""

import math


class FlightError(ValueError):
    """Raised when the one-shot flight sequence is used out of order."""


class OneShotFlightSequence:
    """Small linear sequence which cannot request a second flight cycle."""

    _TRANSITIONS = {
        ("PREFLIGHT", "preflight_ready"): ("ARM", "ARMING"),
        ("ARMING", "armed"): (
            "ENTER_COMMAND_CONTROL", "COMMAND_CONTROL"),
        ("COMMAND_CONTROL", "command_control_ready"): (
            "TAKEOFF", "TAKEOFF"),
        ("TAKEOFF", "airborne"): ("MOVE_VIEW", "AIR_VIEW"),
        ("AIR_VIEW", "at_view"): ("OBSERVE", "AIR_OBSERVE"),
        ("AIR_OBSERVE", "fresh_observation"): (
            "HANDOFF", "AIR_HANDOFF"),
        ("AIR_HANDOFF", "handoff_published"): ("LAND", "LANDING"),
        ("LANDING", "landed"): ("COMPLETE", "COMPLETE"),
    }

    def __init__(self):
        self.phase = "PREFLIGHT"
        self.actions = []

    def advance(self, event):
        try:
            action, next_phase = self._TRANSITIONS[(self.phase, event)]
        except KeyError as error:
            raise FlightError(
                "event %r is invalid in phase %s" %
                (event, self.phase)) from error
        self.actions.append(action)
        self.phase = next_phase
        return action


def _finite_vector(values, size):
    try:
        sequence = tuple(values)
    except TypeError:
        return None
    if len(sequence) != size or any(type(value) is bool for value in sequence):
        return None
    try:
        converted = tuple(float(value) for value in sequence)
    except (TypeError, ValueError, OverflowError):
        return None
    if not all(math.isfinite(value) for value in converted):
        return None
    return converted


def _current_age(age, max_age):
    try:
        age = float(age)
        max_age = float(max_age)
    except (TypeError, ValueError, OverflowError):
        return False
    return (math.isfinite(age) and math.isfinite(max_age)
            and max_age > 0.0 and 0.0 <= age <= max_age)


def message_is_fresh(age, max_age):
    """Check callback-receipt age without depending on ROS message types."""
    return _current_age(age, max_age)


def preflight_ready(connected, odom_valid, armed, failsafe, velocity,
                    state_age, control_age, max_age, max_speed):
    """Return whether the aircraft is stationary and healthy before arming."""
    velocity = _finite_vector(velocity, 3)
    try:
        max_speed = float(max_speed)
    except (TypeError, ValueError, OverflowError):
        return False
    if velocity is None or not math.isfinite(max_speed) or max_speed < 0.0:
        return False
    speed = math.sqrt(sum(value * value for value in velocity))
    return bool(
        connected and odom_valid and not armed and not failsafe
        and speed <= max_speed
        and message_is_fresh(state_age, max_age)
        and message_is_fresh(control_age, max_age))


def position_reached(position, target, velocity, tolerance, max_speed):
    """Check both position tolerance and settling speed."""
    position = _finite_vector(position, 3)
    target = _finite_vector(target, 3)
    velocity = _finite_vector(velocity, 3)
    try:
        tolerance = float(tolerance)
        max_speed = float(max_speed)
    except (TypeError, ValueError, OverflowError):
        return False
    if (position is None or target is None or velocity is None
            or not math.isfinite(tolerance) or tolerance < 0.0
            or not math.isfinite(max_speed) or max_speed < 0.0):
        return False
    error = math.sqrt(sum(
        (actual - desired) ** 2
        for actual, desired in zip(position, target)))
    speed = math.sqrt(sum(value * value for value in velocity))
    return error <= tolerance and speed <= max_speed


def observation_is_fresh(stamp, now, max_age, frame_id, expected_frame,
                         max_future_skew=0.1):
    """Validate that a sensor pose belongs to the current simulation epoch."""
    try:
        stamp = float(stamp)
        now = float(now)
        max_age = float(max_age)
        max_future_skew = float(max_future_skew)
    except (TypeError, ValueError, OverflowError):
        return False
    if not all(math.isfinite(value) for value in (
            stamp, now, max_age, max_future_skew)):
        return False
    age = now - stamp
    return bool(
        stamp > 0.0 and max_age > 0.0 and max_future_skew >= 0.0
        and -max_future_skew <= age <= max_age
        and frame_id == expected_frame and bool(expected_frame))


def safe_stop_actions(armed, command_control):
    """Return the smallest safe terminal action set for the current state."""
    if not armed:
        return ()
    if command_control:
        return ("HOLD", "LAND")
    return ("AUTO_LAND",)
