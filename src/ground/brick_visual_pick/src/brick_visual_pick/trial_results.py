"""Stage tracking and aggregate metrics for visual-pick trials."""
from __future__ import division


class PickProgress(object):
    def __init__(self):
        self.perception_success = False
        self.planning_success = False
        self.grasp_success = False
        self.manipulation_success = False
        self.execution_started = False
        self.failure_type = ''
        self.failure_detail = ''

    def handle_status(self, status):
        if status == 'POSE_PUBLISHED':
            self.perception_success = True
        elif status.startswith('PERCEPTION_REJECTED:'):
            self.failure_type = 'perception_rejected'
            self.failure_detail = status.split(':', 1)[1]

    def handle_log(self, message):
        if any(tag in message for tag in (
                '[OPEN_GRIPPER]', '[PLAN_PREGRASP]', '[MOVE_PREGRASP]',
                '[CARTESIAN_APPROACH]', '[CLOSE_GRIPPER]',
                '[VERIFY_GRASP]', '[LIFT]', '[SUCCESS]')):
            self.execution_started = True
        if '[MOVE_PREGRASP] executing planned trajectory' in message:
            self.planning_success = True
        if ('[VERIFY_GRASP]' in message and
                'attached' in message and
                '[ERROR]' not in message):
            self.grasp_success = True
        if '[SUCCESS] brick pick and lift completed' in message:
            self.manipulation_success = True
        if '[ERROR]' in message and not self.failure_type:
            self.failure_type = 'manipulation_failed'
            self.failure_detail = message.split('[ERROR]', 1)[1].strip()


def is_lifted(initial_height, final_height, minimum_lift):
    return float(final_height) - float(initial_height) >= float(minimum_lift)


def _failure_category(trial):
    if not trial['perception_success']:
        return 'perception_failed'
    if not trial['planning_success']:
        return 'planning_failed'
    if not trial['grasp_success']:
        return 'grasp_failed'
    if not trial['end_to_end_success']:
        return 'lift_failed'
    return ''


def summarize_trials(trials):
    count = len(trials)
    if not count:
        return {
            'trial_count': 0, 'perception_rate': 0.0,
            'planning_rate': 0.0, 'grasp_rate': 0.0,
            'end_to_end_rate': 0.0, 'failure_counts': {},
        }
    failure_counts = {}
    for item in trials:
        category = _failure_category(item)
        if category:
            failure_counts[category] = failure_counts.get(category, 0) + 1
    return {
        'trial_count': count,
        'perception_rate': sum(bool(item['perception_success'])
                               for item in trials) / count,
        'planning_rate': sum(bool(item['planning_success'])
                             for item in trials) / count,
        'grasp_rate': sum(bool(item['grasp_success'])
                          for item in trials) / count,
        'end_to_end_rate': sum(bool(item['end_to_end_success'])
                               for item in trials) / count,
        'failure_counts': failure_counts,
    }
