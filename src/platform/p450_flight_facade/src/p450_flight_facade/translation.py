"""Pure validation and translation for the Prometheus flight facade."""

import math


TAKEOFF = 1
FLY_TO = 2
HOVER = 3
LAND = 4


class TranslationError(ValueError):
    """Raised when a public flight command cannot be translated."""


def _target_tuple(target):
    if target is None:
        raise TranslationError("FLY_TO requires a target")
    try:
        values = tuple(float(value) for value in target)
    except (TypeError, ValueError, OverflowError) as error:
        raise TranslationError("target must contain four finite values") from error
    if (len(values) != 4 or not all(math.isfinite(value) for value in values)):
        raise TranslationError("target must contain four finite values")
    return values


def translation_for(command, target=None):
    """Return the exact native operations for one public command."""
    if command == TAKEOFF:
        if target is not None:
            raise TranslationError("TAKEOFF does not accept a target")
        return (
            ("setup", "ARMING", True),
            ("setup", "SET_CONTROL_MODE", "COMMAND_CONTROL"),
            ("setup", "SET_PX4_MODE", "OFFBOARD"),
            ("command", "Init_Pos_Hover", None),
        )
    if command == FLY_TO:
        return (("command", "Move_XYZ_POS", _target_tuple(target)),)
    if command == HOVER:
        if target is not None:
            raise TranslationError("HOVER does not accept a target")
        return (("command", "Current_Pos_Hover", None),)
    if command == LAND:
        if target is not None:
            raise TranslationError("LAND does not accept a target")
        return (("command", "Land", None),)
    raise TranslationError("unknown public flight command")


def preempt_translation(command):
    """Translate cancellation without inventing backend recovery behavior."""
    if command == FLY_TO:
        return translation_for(HOVER)
    return ()


def backend_healthy(connected, odom_valid, armed, failsafe, state_age,
                    control_age, max_age):
    """Observe whether native Prometheus state is current and usable."""
    del armed  # Arming is command-specific, not a general backend-health bit.
    try:
        state_age = float(state_age)
        control_age = float(control_age)
        max_age = float(max_age)
    except (TypeError, ValueError, OverflowError):
        return False
    return bool(
        connected and odom_valid and not failsafe
        and all(math.isfinite(value) for value in (
            state_age, control_age, max_age))
        and max_age > 0.0
        and 0.0 <= state_age <= max_age
        and 0.0 <= control_age <= max_age)


def position_complete(position, target, velocity, position_tolerance,
                      settle_speed):
    """Observe native position and speed completion for TAKEOFF/FLY_TO."""
    try:
        position = tuple(float(value) for value in position)
        target = tuple(float(value) for value in target)
        velocity = tuple(float(value) for value in velocity)
        position_tolerance = float(position_tolerance)
        settle_speed = float(settle_speed)
    except (TypeError, ValueError, OverflowError):
        return False
    values = position + target + velocity + (
        position_tolerance, settle_speed)
    if (len(position) != 3 or len(target) != 3 or len(velocity) != 3
            or not all(math.isfinite(value) for value in values)
            or position_tolerance < 0.0 or settle_speed < 0.0):
        return False
    error = math.sqrt(sum(
        (actual - desired) ** 2
        for actual, desired in zip(position, target)))
    speed = math.sqrt(sum(value * value for value in velocity))
    return error <= position_tolerance and speed <= settle_speed
