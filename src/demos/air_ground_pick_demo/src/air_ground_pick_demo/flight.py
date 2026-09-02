"""Small validation helpers for the common Prometheus-backed facade."""

import math


def native_state_ready(
        connected, odom_valid, received_age, maximum_age):
    try:
        received_age = float(received_age)
        maximum_age = float(maximum_age)
    except (TypeError, ValueError, OverflowError):
        return False
    return bool(
        connected and odom_valid and
        math.isfinite(received_age) and math.isfinite(maximum_age) and
        maximum_age > 0.0 and 0.0 <= received_age <= maximum_age)


def observation_is_fresh(
        stamp, now, maximum_age, frame_id, expected_frame,
        maximum_future_skew=0.1):
    try:
        stamp = float(stamp)
        now = float(now)
        maximum_age = float(maximum_age)
        maximum_future_skew = float(maximum_future_skew)
    except (TypeError, ValueError, OverflowError):
        return False
    values = (stamp, now, maximum_age, maximum_future_skew)
    age = now - stamp
    return bool(
        all(math.isfinite(value) for value in values) and stamp > 0.0 and
        maximum_age > 0.0 and maximum_future_skew >= 0.0 and
        -maximum_future_skew <= age <= maximum_age and
        frame_id == expected_frame and bool(expected_frame))
