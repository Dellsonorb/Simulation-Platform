"""Pure navigation accuracy and matrix statistics."""
from __future__ import division

import math


def normalize_angle(value):
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi


def pose_error(actual, target):
    dx = float(actual[0]) - float(target[0])
    dy = float(actual[1]) - float(target[1])
    return {
        'xy': math.sqrt(dx * dx + dy * dy),
        'yaw': abs(normalize_angle(float(actual[2]) - float(target[2]))),
    }


def relative_transform_error(reference, current):
    """Return translation and shortest quaternion angular distance."""
    dx = float(current[0][0]) - float(reference[0][0])
    dy = float(current[0][1]) - float(reference[0][1])
    dz = float(current[0][2]) - float(reference[0][2])
    dot = sum(float(a) * float(b)
              for a, b in zip(reference[1], current[1]))
    dot = max(-1.0, min(1.0, abs(dot)))
    return {
        'translation': math.sqrt(dx * dx + dy * dy + dz * dz),
        'rotation': 2.0 * math.acos(dot),
    }


def heading_alignment_evidence(samples, initial_bearing, release_bearing):
    """Evaluate commands while the physical target bearing exceeds release."""
    active = [item for item in samples
              if item.get('bearing') is not None and
              abs(float(item['bearing'])) > float(release_bearing)]
    max_linear = max([abs(float(item['linear']))
                      for item in active] or [0.0])
    turning = [float(item['angular']) for item in active
               if abs(float(item['angular'])) > 1e-9]
    first_angular = turning[0] if turning else 0.0
    return {
        'sample_count': len(active),
        'max_linear': max_linear,
        'first_angular': first_angular,
        'correct_turn_direction': bool(turning) and (
            first_angular * float(initial_bearing) > 0.0),
    }


def evaluate_navigation(action_succeeded, xy_error, yaw_error,
                        linear_speed, angular_speed, tf_preserved,
                        xy_tolerance, yaw_tolerance,
                        stop_linear_tolerance, stop_angular_tolerance):
    failure = ''
    if not action_succeeded:
        failure = 'move_base_failed'
    elif not tf_preserved:
        failure = 'tf_changed'
    elif float(xy_error) > float(xy_tolerance):
        failure = 'xy_error'
    elif float(yaw_error) > float(yaw_tolerance):
        failure = 'yaw_error'
    elif (abs(float(linear_speed)) > float(stop_linear_tolerance) or
          abs(float(angular_speed)) > float(stop_angular_tolerance)):
        failure = 'not_stopped'
    return {'success': not failure, 'failure_type': failure}


def _median(values):
    ordered = sorted(values)
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return 0.5 * (ordered[middle - 1] + ordered[middle])


def summarize_trials(trials):
    count = len(trials)
    successes = sum(bool(item['success']) for item in trials)
    failures = {}
    for item in trials:
        if not item['success']:
            key = item.get('failure_type') or 'unknown'
            failures[key] = failures.get(key, 0) + 1
    xy_values = [float(item['xy_error']) for item in trials]
    yaw_values = [float(item['yaw_error']) for item in trials]
    return {
        'trial_count': count,
        'success_count': successes,
        'success_rate': successes / count if count else 0.0,
        'failure_counts': failures,
        'xy_mean': sum(xy_values) / count if count else None,
        'xy_median': _median(xy_values) if count else None,
        'xy_max': max(xy_values) if count else None,
        'yaw_mean': sum(yaw_values) / count if count else None,
        'yaw_median': _median(yaw_values) if count else None,
        'yaw_max': max(yaw_values) if count else None,
        'worst_xy_case': (max(trials, key=lambda item: item['xy_error'])
                          if count else None),
        'worst_yaw_case': (max(trials, key=lambda item: item['yaw_error'])
                           if count else None),
        'slowest_case': (max(trials, key=lambda item: item['duration'])
                         if count else None),
    }
