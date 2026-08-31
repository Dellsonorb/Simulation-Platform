"""Pure state transitions for the ground Approach-refine-pick mission."""
from __future__ import division

import math


TERMINAL_STATES = ('SUCCEEDED', 'FAILED')


def _finite(value):
    return not math.isnan(float(value)) and not math.isinf(float(value))


class MissionStateMachine(object):
    """Enforce ordering and freshness without depending on ROS callbacks."""

    def __init__(self, stop_settle_time, required_pose_frame):
        self.stop_settle_time = float(stop_settle_time)
        self.required_pose_frame = required_pose_frame
        self.state = 'IDLE'
        self.failure_type = ''
        self.failure_detail = ''
        self.navigation_success = False
        self.perception_success = False
        self.planning_success = False
        self.grasp_success = False
        self.end_to_end_success = False
        self.pick_started = False
        self.started_at = None
        self.navigation_arrived_at = None
        self.stopped_since = None
        self.refine_started_at = None

    @property
    def terminal(self):
        return self.state in TERMINAL_STATES

    def start(self, now):
        if self.state != 'IDLE':
            return False
        self.started_at = float(now)
        self.state = 'WAITING_APPROACH'
        return True

    def fail(self, category, detail):
        if self.terminal:
            return False
        self.failure_type = str(category)
        self.failure_detail = str(detail)
        self.state = 'FAILED'
        return True

    def handle_approach(self, approach_state, reason, now):
        if self.terminal:
            return
        if approach_state == 'NAVIGATING':
            self.state = 'NAVIGATING'
        elif approach_state == 'ARRIVED':
            self.navigation_success = True
            self.navigation_arrived_at = float(now)
            self.stopped_since = None
            self.state = 'VERIFYING_STOP'
        elif approach_state in ('REJECTED', 'NAVIGATION_FAILED'):
            self.fail('navigation_failed', reason or approach_state)

    def handle_stop_sample(self, stopped, now):
        if self.state != 'VERIFYING_STOP':
            return False
        now = float(now)
        if not stopped:
            self.stopped_since = None
            return False
        if self.stopped_since is None:
            self.stopped_since = now
            return False
        if now - self.stopped_since + 1e-9 < self.stop_settle_time:
            return False
        self.state = 'REFINE_READY'
        return True

    def refine_started(self, now):
        if self.state != 'REFINE_READY':
            return False
        self.refine_started_at = float(now)
        self.state = 'MOVING_OBSERVATION'
        return True

    def observation_finished(self, success, detail):
        if self.state != 'MOVING_OBSERVATION' or self.terminal:
            return False
        if not success:
            self.fail('observation_failed', detail or 'observation_failed')
            return False
        self.state = 'REFINING'
        return True

    def handle_refine_status(self, status):
        if self.terminal:
            return
        if status.startswith('PERCEPTION_REJECTED:'):
            self.fail('perception_rejected', status.split(':', 1)[1])

    def accept_refined_pose(self, stamp, frame_id):
        if self.state != 'REFINING' or self.terminal:
            return False
        if not _finite(stamp) or self.refine_started_at is None or \
                float(stamp) <= self.refine_started_at:
            self.fail('stale_refined_pose', 'pose_not_after_refine_start')
            return False
        if frame_id != self.required_pose_frame:
            self.fail('wrong_refined_pose_frame', frame_id)
            return False
        self.perception_success = True
        self.state = 'PICKING'
        return True

    def begin_pick_execution(self):
        """Mark the one-way handoff only when controllers are ready."""
        if self.state != 'PICKING' or self.terminal or self.pick_started:
            return False
        self.pick_started = True
        return True

    def _pick_failure_category(self, message):
        lowered = message.lower()
        if 'lift' in lowered:
            return 'lift_failed'
        if ('planning' in lowered or 'pre-grasp execution' in lowered or
                'cartesian approach' in lowered):
            return 'planning_failed'
        if ('gripper' in lowered or 'attachment' in lowered or
                'grasp' in lowered):
            return 'grasp_failed'
        return 'manipulation_failed'

    def handle_pick_log(self, message):
        if self.state not in ('PICKING', 'VERIFYING_LIFT') or self.terminal:
            return
        if '[MOVE_PREGRASP] executing planned trajectory' in message:
            self.planning_success = True
        if ('[VERIFY_GRASP]' in message and 'attached' in message and
                '[ERROR]' not in message):
            self.grasp_success = True
        if '[ERROR]' in message:
            self.fail(self._pick_failure_category(message),
                      message.split('[ERROR]', 1)[1].strip())
        elif '[SUCCESS] brick pick and lift completed' in message:
            self.state = 'VERIFYING_LIFT'

    def verify_lift(self, lifted):
        if self.state != 'VERIFYING_LIFT' or self.terminal:
            return False
        if not lifted:
            self.fail('lift_failed', 'physical_lift_not_verified')
            return False
        self.end_to_end_success = True
        self.state = 'SUCCEEDED'
        return True

    def timeout(self, stage):
        return self.fail('{}_timeout'.format(stage), stage)
