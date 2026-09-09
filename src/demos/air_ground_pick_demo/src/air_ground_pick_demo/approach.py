"""Backend-neutral standoff geometry for the common navigation action."""

from collections import namedtuple
import math


class ApproachError(ValueError):
    pass


StandoffGoal = namedtuple("StandoffGoal", "x y yaw target_distance")
GoalGeometry = namedtuple("GoalGeometry", "x y yaw")


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


def _finite_pose(values, name):
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError, OverflowError) as error:
        raise ApproachError(
            "%s must contain three finite values" % name) from error
    if (len(result) != 3 or
            not all(math.isfinite(value) for value in result)):
        raise ApproachError("%s must contain three finite values" % name)
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


def compute_staged_standoff_goals(current_pose, target_xy, standoff):
    """Return a precise positioning goal followed by the final heading."""
    current_x, current_y, current_yaw = _finite_pose(
        current_pose, "current_pose")
    final = compute_standoff_goal(
        (current_x, current_y), target_xy, standoff)
    positioning = StandoffGoal(
        final.x, final.y, current_yaw, final.target_distance)
    return positioning, final


def compute_staged_candidate_goals(current_pose, candidate_pose):
    """Keep an RM4D base candidate exactly unchanged for navigation."""
    _finite_pose(current_pose, "current_pose")
    candidate = GoalGeometry(
        *_finite_pose(candidate_pose, "candidate_pose"))
    return candidate, candidate


def compute_heading_goal(current_pose, target_xy):
    """Keep the measured base position fixed and face the target."""
    current_x, current_y, _current_yaw = _finite_pose(
        current_pose, "current_pose")
    target_x, target_y = _finite_pair(target_xy, "target_xy")
    delta_x = target_x - current_x
    delta_y = target_y - current_y
    distance = math.hypot(delta_x, delta_y)
    if distance <= 1e-6:
        raise ApproachError(
            "current base position cannot equal the target position")
    return StandoffGoal(
        current_x, current_y, math.atan2(delta_y, delta_x), distance)


def motion_required(current_xy, goal_xy, tolerance):
    try:
        tolerance = float(tolerance)
    except (TypeError, ValueError, OverflowError) as error:
        raise ApproachError("motion tolerance must be nonnegative") from error
    if not math.isfinite(tolerance) or tolerance < 0.0:
        raise ApproachError("motion tolerance must be nonnegative")
    return travel_distance(current_xy, goal_xy) > tolerance


def arrival_within_tolerance(actual, goal, xy_tolerance, yaw_tolerance):
    actual, goal = _finite_pose(actual, 'actual'), _finite_pose(goal, 'goal')
    xy_limit = _positive(xy_tolerance, 'xy_tolerance')
    yaw_limit = _positive(yaw_tolerance, 'yaw_tolerance')
    yaw_error = abs(math.atan2(math.sin(actual[2]-goal[2]), math.cos(actual[2]-goal[2])))
    return travel_distance(actual[:2], goal[:2]) <= xy_limit and yaw_error <= yaw_limit


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
