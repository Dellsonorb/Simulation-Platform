import math


ZERO = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def command_is_fresh(received_at, now, timeout):
    try:
        received_at = float(received_at)
        now = float(now)
        timeout = float(timeout)
    except (TypeError, ValueError, OverflowError):
        return False
    if not all(math.isfinite(value) for value in (
            received_at, now, timeout)) or timeout <= 0.0:
        return False
    age = now - received_at
    return 0.0 <= age <= timeout


def admit_components(linear_x, linear_y, linear_z,
                     angular_x, angular_y, angular_z,
                     epsilon=1e-9):
    raw = (linear_x, linear_y, linear_z,
           angular_x, angular_y, angular_z)
    if any(type(value) is bool for value in raw):
        return ZERO, "nonfinite"
    try:
        values = tuple(float(value) for value in raw)
    except (TypeError, ValueError, OverflowError):
        return ZERO, "nonfinite"
    if not all(math.isfinite(value) for value in values):
        return ZERO, "nonfinite"
    if any(abs(value) > epsilon for value in
           (values[1], values[2], values[3], values[4])):
        return ZERO, "nonplanar"
    admitted = (
        max(-0.5, min(0.5, values[0])), 0.0, 0.0, 0.0, 0.0,
        max(-1.0, min(1.0, values[5])),
    )
    return admitted, ("clamped" if admitted[0] != values[0]
                      or admitted[5] != values[5] else None)
