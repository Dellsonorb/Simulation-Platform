"""Pure helpers for repeatable multi-pose robustness validation."""
from __future__ import division

import math

import numpy as np

from brick_rgbd_perception.pose_estimation import yaw_error_mod_pi


REQUIRED_CATEGORIES = frozenset((
    'center', 'workspace_edge', 'image_boundary', 'expected_rejection'))
METRIC_NAMES = ('xy', 'z', 'position', 'yaw')


def select_stamped_samples(samples, count, minimum_span):
    """Select unique timestamped samples only when they cover enough time."""
    unique = []
    seen = set()
    for stamp, value in sorted(samples, key=lambda item: item[0]):
        if stamp in seen:
            continue
        seen.add(stamp)
        unique.append((stamp, value))
    count = int(count)
    for start in range(len(unique) - count + 1):
        window = unique[start:start + count]
        if window[-1][0] - window[0][0] >= float(minimum_span):
            return window
    return None


def observation_joint_error(actual, expected):
    """Maximum absolute error for a complete named joint observation."""
    missing = set(expected) - set(actual)
    if missing:
        raise ValueError('missing observation joints: %s' %
                         ', '.join(sorted(missing)))
    return max(abs(float(actual[name]) - float(target))
               for name, target in expected.items())


def pose_errors(estimate_position, estimate_yaw,
                truth_position, truth_yaw):
    """Return metric errors in metres/radians for one 4DoF pose pair."""
    estimate = np.asarray(estimate_position, dtype=np.float64)
    truth = np.asarray(truth_position, dtype=np.float64)
    if estimate.shape != (3,) or truth.shape != (3,):
        raise ValueError('positions must contain exactly three values')
    delta = estimate - truth
    return {
        'xy': float(np.linalg.norm(delta[:2])),
        'z': float(abs(delta[2])),
        'position': float(np.linalg.norm(delta)),
        'yaw': float(yaw_error_mod_pi(estimate_yaw, truth_yaw)),
    }


def representative_pose(samples):
    """Return a robust position/yaw and maximum multi-frame deviations."""
    if not samples:
        raise ValueError('at least one pose sample is required')
    positions = np.asarray([sample[0] for sample in samples],
                           dtype=np.float64)
    yaws = np.asarray([sample[1] for sample in samples], dtype=np.float64)
    if positions.ndim != 2 or positions.shape[1] != 3:
        raise ValueError('sample positions must contain three values')
    position = np.median(positions, axis=0)
    # Doubling maps axes that differ by pi onto the same directed angle.
    yaw = 0.5 * math.atan2(np.sin(2.0 * yaws).mean(),
                           np.cos(2.0 * yaws).mean())
    position_deviation = np.linalg.norm(positions - position, axis=1)
    yaw_deviation = np.asarray([yaw_error_mod_pi(value, yaw)
                                for value in yaws])
    return position, yaw, {
        'position_max': float(position_deviation.max()),
        'yaw_max': float(yaw_deviation.max()),
    }


def summarize_scenarios(results):
    """Summarize one representative error value per successful scenario."""
    total = len(results)
    successful = [result for result in results
                  if result.get('outcome') == 'success']
    summary = {
        'total': total,
        'successes': len(successful),
        'rejections': total - len(successful),
        'success_rate': len(successful) / float(total) if total else 0.0,
        'rejection_rate': ((total - len(successful)) / float(total)
                           if total else 0.0),
        'metrics': {},
        'categories': {},
    }
    expected_matches = sum(
        1 for result in results
        if result.get('outcome') == result.get('expect'))
    summary['expected_match_rate'] = (
        expected_matches / float(total) if total else 0.0)
    for category in sorted(set(result.get('category') for result in results)):
        category_results = [result for result in results
                            if result.get('category') == category]
        category_successes = sum(
            1 for result in category_results
            if result.get('outcome') == 'success')
        summary['categories'][category] = {
            'total': len(category_results),
            'successes': category_successes,
            'rejections': len(category_results) - category_successes,
        }
    for name in METRIC_NAMES:
        values = np.asarray([result['errors'][name]
                             for result in successful], dtype=np.float64)
        if not values.size:
            summary['metrics'][name] = {
                'mean': None, 'median': None, 'max': None,
                'worst_index': None}
            continue
        summary['metrics'][name] = {
            'mean': float(values.mean()),
            'median': float(np.median(values)),
            'max': float(values.max()),
            'worst_index': int(successful[int(np.argmax(values))].get(
                'index', int(np.argmax(values)))),
        }
    return summary


def validate_scenarios(scenarios):
    """Reject malformed scenario sets before Gazebo is changed."""
    if not 20 <= len(scenarios) <= 30:
        raise ValueError('validation requires 20 to 30 scenarios')
    identifiers = [scenario.get('id') for scenario in scenarios]
    if any(not identifier for identifier in identifiers):
        raise ValueError('every scenario requires an id')
    if len(set(identifiers)) != len(identifiers):
        raise ValueError('scenario ids must be unique')
    categories = set(scenario.get('category') for scenario in scenarios)
    if categories != REQUIRED_CATEGORIES:
        raise ValueError('scenario set is not stratified across all categories')
    coordinates = []
    for scenario in scenarios:
        if scenario.get('expect') not in ('success', 'rejected'):
            raise ValueError('scenario expectation must be success or rejected')
        try:
            coordinate = tuple(float(scenario[name])
                               for name in ('x', 'y', 'yaw'))
        except (KeyError, TypeError, ValueError):
            raise ValueError('scenario x, y and yaw must be numeric')
        if not np.all(np.isfinite(coordinate)):
            raise ValueError('scenario x, y and yaw must be finite')
        coordinates.append(coordinate)
    if len(set(coordinates)) != len(coordinates):
        raise ValueError('scenario poses must be unique')
