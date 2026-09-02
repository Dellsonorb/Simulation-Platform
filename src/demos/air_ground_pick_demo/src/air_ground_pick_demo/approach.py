"""Backend-neutral standoff geometry for the common navigation action."""

from collections import namedtuple
import math


class ApproachError(ValueError):
    pass


StandoffGoal = namedtuple("StandoffGoal", "x y yaw target_distance")


def _finite_pair(values, name):
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError, OverflowError) as error:
        raise ApproachError("%s must contain two finite values" % name) \
            from error
    if (len(result) != 2 or
            not all(math.isfinite(value) for value in result)):
        raise ApproachError("%s must contain two finite values" % name)
    return result


def _positive(value, name):
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ApproachError("%s must be positive" % name) from error
    if not math.isfinite(result) or result <= 0.0:
        raise ApproachError("%s must be positive" % name)
    return result


def compute_standoff_goal(current_xy, target_xy, standoff):
    """Place the base on its current side of the target and face the target."""
    current_x, current_y = _finite_pair(current_xy, "current_xy")
    target_x, target_y = _finite_pair(target_xy, "target_xy")
    standoff = _positive(standoff, "standoff")
    from_target_x = current_x - target_x
    from_target_y = current_y - target_y
    distance = math.hypot(from_target_x, from_target_y)
    if distance <= 1e-6:
        raise ApproachError(
            "current base position cannot equal the target position")
    goal_x = target_x + standoff * from_target_x / distance
    goal_y = target_y + standoff * from_target_y / distance
    yaw = math.atan2(target_y - goal_y, target_x - goal_x)
    return StandoffGoal(goal_x, goal_y, yaw, standoff)


def motion_required(current_xy, goal_xy, tolerance):
    try:
        tolerance = float(tolerance)
    except (TypeError, ValueError, OverflowError) as error:
        raise ApproachError("motion tolerance must be nonnegative") from error
    if not math.isfinite(tolerance) or tolerance < 0.0:
        raise ApproachError("motion tolerance must be nonnegative")
    return travel_distance(current_xy, goal_xy) > tolerance


def travel_distance(initial_xy, current_xy):
    initial_x, initial_y = _finite_pair(initial_xy, "initial_xy")
    current_x, current_y = _finite_pair(current_xy, "current_xy")
    return math.hypot(current_x - initial_x, current_y - initial_y)


def transform_is_fresh(stamp, now, max_age, max_future_skew=0.1):
    try:
        values = tuple(float(value) for value in (
            stamp, now, max_age, max_future_skew))
    except (TypeError, ValueError, OverflowError):
        return False
    stamp, now, max_age, max_future_skew = values
    age = now - stamp
    return bool(
        all(math.isfinite(value) for value in values) and stamp > 0.0 and
        max_age > 0.0 and max_future_skew >= 0.0 and
        -max_future_skew <= age <= max_age)
