"""Pure geometry and LiDAR guards for the bounded BUNKER approach."""

from collections import namedtuple
import math


class ApproachError(ValueError):
    """Raised for invalid controller or scan inputs."""


_LimitFields = namedtuple(
    "_LimitFields",
    ("standoff", "distance_tolerance", "heading_tolerance",
     "turn_in_place_angle", "linear_gain", "angular_gain",
     "max_linear", "max_angular", "obstacle_stop_distance"))


class ApproachLimits(_LimitFields):
    """Validated constants for the deliberately small approach controller."""

    __slots__ = ()

    def __new__(cls, standoff, distance_tolerance, heading_tolerance,
                turn_in_place_angle, linear_gain, angular_gain,
                max_linear, max_angular, obstacle_stop_distance):
        raw = (standoff, distance_tolerance, heading_tolerance,
               turn_in_place_angle, linear_gain, angular_gain,
               max_linear, max_angular, obstacle_stop_distance)
        if any(type(value) is bool for value in raw):
            raise ApproachError("controller limits must be finite numbers")
        try:
            values = tuple(float(value) for value in raw)
        except (TypeError, ValueError, OverflowError) as error:
            raise ApproachError(
                "controller limits must be finite numbers") from error
        if not all(math.isfinite(value) for value in values):
            raise ApproachError("controller limits must be finite numbers")
        (standoff, distance_tolerance, heading_tolerance,
         turn_in_place_angle, linear_gain, angular_gain,
         max_linear, max_angular, obstacle_stop_distance) = values
        if (standoff <= 0.0 or distance_tolerance < 0.0
                or not 0.0 <= heading_tolerance <= math.pi
                or not 0.0 < turn_in_place_angle <= math.pi
                or linear_gain <= 0.0 or angular_gain <= 0.0
                or max_linear <= 0.0 or max_angular <= 0.0
                or obstacle_stop_distance <= 0.0
                or standoff <= obstacle_stop_distance):
            raise ApproachError("unsafe controller limits")
        return super().__new__(
            cls, standoff, distance_tolerance, heading_tolerance,
            turn_in_place_angle, linear_gain, angular_gain,
            max_linear, max_angular, obstacle_stop_distance)


ApproachCommand = namedtuple(
    "ApproachCommand",
    ("linear_x", "angular_z", "reached", "blocked", "distance",
     "heading_error", "forward_clearance"))


def _finite_pair(values, name):
    try:
        values = tuple(values)
    except TypeError as error:
        raise ApproachError("%s must contain two values" % name) from error
    if len(values) != 2 or any(type(value) is bool for value in values):
        raise ApproachError("%s must contain two finite values" % name)
    try:
        values = tuple(float(value) for value in values)
    except (TypeError, ValueError, OverflowError) as error:
        raise ApproachError(
            "%s must contain two finite values" % name) from error
    if not all(math.isfinite(value) for value in values):
        raise ApproachError("%s must contain two finite values" % name)
    return values


def wrap_angle(angle):
    try:
        angle = float(angle)
    except (TypeError, ValueError, OverflowError) as error:
        raise ApproachError("angle must be finite") from error
    if not math.isfinite(angle):
        raise ApproachError("angle must be finite")
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def world_to_base(target_xy, base_xy_yaw):
    """Transform one world XY point into a planar robot base frame."""
    target_x, target_y = _finite_pair(target_xy, "target_xy")
    try:
        base_x, base_y, base_yaw = tuple(base_xy_yaw)
        base_x, base_y, base_yaw = (
            float(base_x), float(base_y), float(base_yaw))
    except (TypeError, ValueError, OverflowError) as error:
        raise ApproachError(
            "base_xy_yaw must contain three finite values") from error
    if not all(math.isfinite(value) for value in (
            base_x, base_y, base_yaw)):
        raise ApproachError("base_xy_yaw must contain three finite values")
    delta_x = target_x - base_x
    delta_y = target_y - base_y
    cosine = math.cos(base_yaw)
    sine = math.sin(base_yaw)
    return (
        cosine * delta_x + sine * delta_y,
        -sine * delta_x + cosine * delta_y)


def forward_clearance(ranges, angle_min, angle_increment, sector_half_angle,
                      range_min=0.0, range_max=float("inf")):
    """Return the nearest usable forward-sector LiDAR range."""
    try:
        angle_min = float(angle_min)
        angle_increment = float(angle_increment)
        sector_half_angle = float(sector_half_angle)
        range_min = float(range_min)
        range_max = float(range_max)
    except (TypeError, ValueError, OverflowError) as error:
        raise ApproachError("invalid scan geometry") from error
    if (not all(math.isfinite(value) for value in (
            angle_min, angle_increment, sector_half_angle, range_min))
            or angle_increment <= 0.0 or sector_half_angle <= 0.0
            or range_min < 0.0 or math.isnan(range_max)
            or range_max <= range_min):
        raise ApproachError("invalid scan geometry")
    candidates = []
    for index, raw_range in enumerate(ranges):
        angle = wrap_angle(angle_min + index * angle_increment)
        if abs(angle) > sector_half_angle:
            continue
        if type(raw_range) is bool:
            continue
        try:
            distance = float(raw_range)
        except (TypeError, ValueError, OverflowError):
            continue
        if math.isnan(distance) or distance < range_min:
            continue
        if math.isinf(distance) or distance >= range_max:
            candidates.append(float("inf"))
        elif distance > 0.0:
            candidates.append(distance)
    if not candidates:
        raise ApproachError("forward LiDAR sector has no usable samples")
    return min(candidates)


def _stamp_is_fresh(stamp, now, max_age, max_future_skew):
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
    return bool(stamp > 0.0 and max_age > 0.0
                and max_future_skew >= 0.0
                and -max_future_skew <= age <= max_age)


def scan_is_fresh(stamp, now, max_age, max_future_skew=0.1):
    return _stamp_is_fresh(stamp, now, max_age, max_future_skew)


def transform_is_fresh(stamp, now, max_age, max_future_skew=0.1):
    return _stamp_is_fresh(stamp, now, max_age, max_future_skew)


def clearance_for_status(clearance):
    """Represent an open LiDAR ray as JSON null instead of nonfinite inf."""
    try:
        clearance = float(clearance)
    except (TypeError, ValueError, OverflowError) as error:
        raise ApproachError("clearance status must be nonnegative") from error
    if math.isnan(clearance) or clearance < 0.0:
        raise ApproachError("clearance status must be nonnegative")
    return clearance if math.isfinite(clearance) else None


def _clamp(value, magnitude):
    return max(-magnitude, min(magnitude, value))


def compute_command(target_x, target_y, forward_clearance, limits):
    """Compute one bounded differential-drive command toward the target."""
    if not isinstance(limits, ApproachLimits):
        raise ApproachError("limits must be an ApproachLimits instance")
    target_x, target_y = _finite_pair(
        (target_x, target_y), "target position")
    try:
        forward_clearance = float(forward_clearance)
    except (TypeError, ValueError, OverflowError) as error:
        raise ApproachError("forward clearance must be nonnegative") from error
    if math.isnan(forward_clearance) or forward_clearance < 0.0:
        raise ApproachError("forward clearance must be nonnegative")

    distance = math.hypot(target_x, target_y)
    heading = math.atan2(target_y, target_x) if distance > 0.0 else 0.0
    reached = bool(
        distance <= limits.standoff + limits.distance_tolerance
        and abs(heading) <= limits.heading_tolerance)
    if reached:
        return ApproachCommand(
            0.0, 0.0, True, False, distance, heading,
            forward_clearance)
    if forward_clearance <= limits.obstacle_stop_distance:
        return ApproachCommand(
            0.0, 0.0, False, True, distance, heading,
            forward_clearance)

    angular = _clamp(
        limits.angular_gain * heading, limits.max_angular)
    linear = 0.0
    if (abs(heading) <= limits.turn_in_place_angle
            and distance > limits.standoff):
        distance_error = distance - limits.standoff
        linear = min(
            limits.max_linear,
            limits.linear_gain * distance_error * max(0.0, math.cos(heading)))
    return ApproachCommand(
        linear, angular, False, False, distance, heading,
        forward_clearance)


def travel_distance(initial_xy, current_xy):
    initial_x, initial_y = _finite_pair(initial_xy, "initial_xy")
    current_x, current_y = _finite_pair(current_xy, "current_xy")
    return math.hypot(current_x - initial_x, current_y - initial_y)
