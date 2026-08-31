"""Aggregate repeatable Approach-refine-pick Gazebo trial results."""
from __future__ import division


def _rate(trials, key):
    if not trials:
        return 0.0
    return sum(bool(item.get(key)) for item in trials) / len(trials)


def _control_is_safe(trial):
    if trial.get('container_exit_code') != 0:
        return False
    if not trial.get('safe_stop'):
        return False
    if trial.get('failure_type') != trial.get('expected_failure'):
        return False
    if trial.get('expected_failure') in (
            'navigation_failed', 'observation_failed',
            'perception_rejected') and trial.get('pick_started'):
        return False
    return True


def summarize_trials(trials):
    valid = [item for item in trials if item.get('expect_success')]
    controls = [item for item in trials if not item.get('expect_success')]
    failures = {}
    for item in valid:
        category = item.get('failure_type', '')
        if category:
            failures[category] = failures.get(category, 0) + 1
    slowest = max(valid, key=lambda item: item.get('duration', 0.0)) \
        if valid else None
    return {
        'trial_count': len(trials),
        'valid_trial_count': len(valid),
        'control_count': len(controls),
        'navigation_rate': _rate(valid, 'navigation_success'),
        'refine_perception_rate': _rate(valid, 'perception_success'),
        'planning_rate': _rate(valid, 'planning_success'),
        'grasp_rate': _rate(valid, 'grasp_success'),
        'end_to_end_rate': _rate(valid, 'end_to_end_success'),
        'fresh_pose_rate': _rate(valid, 'fresh_pose_after_stop'),
        'safe_stop_rate': _rate(valid, 'safe_stop'),
        'safe_control_count': sum(_control_is_safe(item)
                                  for item in controls),
        'failure_counts': failures,
        'slowest_case': slowest,
        'all_containers_zero': all(
            item.get('container_exit_code') == 0 for item in trials),
    }


def acceptance_passed(summary, minimum_valid, expected_controls):
    return (
        summary['valid_trial_count'] >= int(minimum_valid) and
        summary['control_count'] == int(expected_controls) and
        summary['safe_control_count'] == int(expected_controls) and
        summary['all_containers_zero'] and
        all(summary[key] == 1.0 for key in (
            'navigation_rate', 'refine_perception_rate', 'planning_rate',
            'grasp_rate', 'end_to_end_rate', 'fresh_pose_rate',
            'safe_stop_rate')))
