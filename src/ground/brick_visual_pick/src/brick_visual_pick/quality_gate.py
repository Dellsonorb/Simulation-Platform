"""Pure multi-frame quality gate for brick pose estimates."""
from __future__ import division

import math


def normalize_yaw_mod_pi(yaw):
    return (float(yaw) + math.pi / 2.0) % math.pi - math.pi / 2.0


def yaw_error_mod_pi(first, second):
    return abs(normalize_yaw_mod_pi(float(first) - float(second)))


def _isfinite(value):
    value = float(value)
    return not math.isnan(value) and not math.isinf(value)


def _median(values):
    ordered = sorted(float(value) for value in values)
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return 0.5 * (ordered[middle - 1] + ordered[middle])


def _mean_axis_yaw(samples):
    sine = sum(math.sin(2.0 * item['yaw']) for item in samples)
    cosine = sum(math.cos(2.0 * item['yaw']) for item in samples)
    return normalize_yaw_mod_pi(0.5 * math.atan2(sine, cosine))


def evaluate_pose_samples(samples, required_count, minimum_span,
                          position_tolerance, yaw_tolerance,
                          required_frame):
    """Return a robust pose only when unique estimates are stable."""
    unique = {}
    for item in samples:
        unique[float(item['stamp'])] = item
    ordered = [unique[key] for key in sorted(unique)]
    if len(ordered) < int(required_count):
        return {'accepted': False,
                'reason': 'insufficient_unique_samples'}
    selected = ordered[-int(required_count):]
    if any(not all(_isfinite(item[key])
                   for key in ('stamp', 'x', 'y', 'z', 'yaw'))
           for item in selected):
        return {'accepted': False, 'reason': 'non_finite_sample'}
    if any(item['frame_id'] != required_frame for item in selected):
        return {'accepted': False, 'reason': 'wrong_frame'}
    if selected[-1]['stamp'] - selected[0]['stamp'] < float(minimum_span):
        return {'accepted': False, 'reason': 'insufficient_time_span'}

    pose = {
        'x': _median([item['x'] for item in selected]),
        'y': _median([item['y'] for item in selected]),
        'z': _median([item['z'] for item in selected]),
        'yaw': _mean_axis_yaw(selected),
    }
    maximum_position_error = max(
        math.sqrt((item['x'] - pose['x']) ** 2 +
                  (item['y'] - pose['y']) ** 2 +
                  (item['z'] - pose['z']) ** 2)
        for item in selected)
    if maximum_position_error > float(position_tolerance):
        return {'accepted': False, 'reason': 'position_unstable',
                'position_spread': maximum_position_error}
    maximum_yaw_error = max(yaw_error_mod_pi(item['yaw'], pose['yaw'])
                            for item in selected)
    if maximum_yaw_error > float(yaw_tolerance):
        return {'accepted': False, 'reason': 'yaw_unstable',
                'yaw_spread': maximum_yaw_error}
    return {
        'accepted': True, 'reason': 'accepted', 'pose': pose,
        'position_spread': maximum_position_error,
        'yaw_spread': maximum_yaw_error,
        'sample_count': len(selected),
        'time_span': selected[-1]['stamp'] - selected[0]['stamp'],
    }
