"""Aggregate end-to-end visual-pick validation results."""
from __future__ import division

from brick_visual_pick.trial_results import summarize_trials


def build_matrix_summary(trials):
    pose_trials = [item for item in trials if item.get('pose_error')]
    failures = [
        {'scenario_id': item['scenario_id'],
         'failure_type': item.get('failure_type', ''),
         'failure_detail': item.get('failure_detail', '')}
        for item in trials if not item['end_to_end_success']
    ]
    return {
        'rates': summarize_trials(trials),
        'failed_cases': failures,
        'worst_pose_case': (max(
            pose_trials, key=lambda item: item['pose_error']['position'])
                            if pose_trials else None),
        'slowest_case': (max(trials, key=lambda item: item['wall_duration'])
                         if trials else None),
    }


def acceptance_passed(summary, expected_count, control):
    rates = summary['rates']
    return (int(expected_count) >= 20 and
            rates['trial_count'] >= 20 and
            rates['trial_count'] == int(expected_count) and
            rates['end_to_end_rate'] == 1.0 and
            bool(control) and bool(control.get('control_safe')))
