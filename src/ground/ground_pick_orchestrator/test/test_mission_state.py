from __future__ import division

import unittest

from ground_pick_orchestrator.mission_state import MissionStateMachine


class MissionStateMachineTest(unittest.TestCase):
    def setUp(self):
        self.machine = MissionStateMachine(
            stop_settle_time=1.0,
            required_pose_frame='aubo_i5_base_link')

    def navigate_and_stop(self):
        self.machine.start(1.0)
        self.machine.handle_approach('NAVIGATING', '', 2.0)
        self.machine.handle_approach('ARRIVED', '', 5.0)
        self.assertFalse(self.machine.handle_stop_sample(True, 5.1))
        self.assertTrue(self.machine.handle_stop_sample(True, 6.1))
        self.machine.refine_started(6.2)

    def test_happy_path_requires_post_navigation_refine_and_lift(self):
        self.navigate_and_stop()
        self.machine.observation_finished(True, '')
        self.machine.handle_refine_status('WAITING_ESTIMATE')
        self.assertTrue(self.machine.accept_refined_pose(
            7.0, 'aubo_i5_base_link'))
        self.assertFalse(self.machine.pick_started)
        self.assertTrue(self.machine.begin_pick_execution())
        self.machine.handle_pick_log(
            '[MOVE_PREGRASP] executing planned trajectory')
        self.machine.handle_pick_log(
            '[VERIFY_GRASP] brick attached to gripper')
        self.machine.handle_pick_log(
            '[SUCCESS] brick pick and lift completed')
        self.machine.verify_lift(True)
        self.assertEqual('SUCCEEDED', self.machine.state)
        self.assertTrue(self.machine.navigation_success)
        self.assertTrue(self.machine.perception_success)
        self.assertTrue(self.machine.planning_success)
        self.assertTrue(self.machine.grasp_success)
        self.assertTrue(self.machine.end_to_end_success)

    def test_stop_window_must_be_continuous(self):
        self.machine.start(1.0)
        self.machine.handle_approach('NAVIGATING', '', 2.0)
        self.machine.handle_approach('ARRIVED', '', 5.0)
        self.assertFalse(self.machine.handle_stop_sample(True, 5.1))
        self.assertFalse(self.machine.handle_stop_sample(False, 5.8))
        self.assertFalse(self.machine.handle_stop_sample(True, 6.0))
        self.assertFalse(self.machine.handle_stop_sample(True, 6.9))
        self.assertTrue(self.machine.handle_stop_sample(True, 7.0))

    def test_navigation_failure_is_terminal(self):
        self.machine.start(1.0)
        self.machine.handle_approach('NAVIGATION_FAILED', 'TIMEOUT', 4.0)
        self.assertEqual('FAILED', self.machine.state)
        self.assertEqual('navigation_failed', self.machine.failure_type)
        self.assertEqual('TIMEOUT', self.machine.failure_detail)
        self.assertFalse(self.machine.handle_stop_sample(True, 6.0))

    def test_approach_rejection_is_navigation_failure(self):
        self.machine.start(1.0)
        self.machine.handle_approach('REJECTED', 'NO_SAFE_CANDIDATE', 2.0)
        self.assertEqual('FAILED', self.machine.state)
        self.assertEqual('navigation_failed', self.machine.failure_type)

    def test_observation_failure_stops_before_perception(self):
        self.navigate_and_stop()
        self.machine.observation_finished(False, 'trajectory_failed')
        self.assertEqual('FAILED', self.machine.state)
        self.assertEqual('observation_failed', self.machine.failure_type)
        self.assertFalse(self.machine.perception_success)

    def test_perception_rejection_stops_before_pick(self):
        self.navigate_and_stop()
        self.machine.observation_finished(True, '')
        self.machine.handle_refine_status(
            'PERCEPTION_REJECTED:timeout_no_estimate')
        self.assertEqual('FAILED', self.machine.state)
        self.assertEqual('perception_rejected', self.machine.failure_type)
        self.assertFalse(self.machine.pick_started)

    def test_pose_older_than_refine_start_is_rejected(self):
        self.navigate_and_stop()
        self.machine.observation_finished(True, '')
        self.assertFalse(self.machine.accept_refined_pose(
            6.1, 'aubo_i5_base_link'))
        self.assertEqual('FAILED', self.machine.state)
        self.assertEqual('stale_refined_pose', self.machine.failure_type)
        self.assertFalse(self.machine.pick_started)

    def test_wrong_pose_frame_is_rejected(self):
        self.navigate_and_stop()
        self.machine.observation_finished(True, '')
        self.assertFalse(self.machine.accept_refined_pose(7.0, 'odom'))
        self.assertEqual('wrong_refined_pose_frame',
                         self.machine.failure_type)

    def test_planning_error_is_classified_and_terminal(self):
        self.navigate_and_stop()
        self.machine.observation_finished(True, '')
        self.assertTrue(self.machine.accept_refined_pose(
            7.0, 'aubo_i5_base_link'))
        self.machine.begin_pick_execution()
        self.machine.handle_pick_log(
            '[ERROR] pre-grasp planning failed')
        self.assertEqual('FAILED', self.machine.state)
        self.assertEqual('planning_failed', self.machine.failure_type)
        self.machine.handle_pick_log(
            '[SUCCESS] brick pick and lift completed')
        self.assertEqual('FAILED', self.machine.state)

    def test_grasp_and_lift_errors_have_distinct_categories(self):
        self.navigate_and_stop()
        self.machine.observation_finished(True, '')
        self.machine.accept_refined_pose(7.0, 'aubo_i5_base_link')
        self.machine.begin_pick_execution()
        self.machine.handle_pick_log(
            '[ERROR] grasp attachment/verification failed')
        self.assertEqual('grasp_failed', self.machine.failure_type)

        other = MissionStateMachine(1.0, 'aubo_i5_base_link')
        other.start(1.0)
        other.handle_approach('NAVIGATING', '', 2.0)
        other.handle_approach('ARRIVED', '', 3.0)
        other.handle_stop_sample(True, 3.1)
        other.handle_stop_sample(True, 4.1)
        other.refine_started(4.2)
        other.observation_finished(True, '')
        other.accept_refined_pose(5.0, 'aubo_i5_base_link')
        other.begin_pick_execution()
        other.handle_pick_log('[ERROR] Cartesian lift failed')
        self.assertEqual('lift_failed', other.failure_type)

    def test_success_log_still_requires_physical_lift(self):
        self.navigate_and_stop()
        self.machine.observation_finished(True, '')
        self.machine.accept_refined_pose(7.0, 'aubo_i5_base_link')
        self.machine.begin_pick_execution()
        self.machine.handle_pick_log(
            '[SUCCESS] brick pick and lift completed')
        self.assertEqual('VERIFYING_LIFT', self.machine.state)
        self.machine.verify_lift(False)
        self.assertEqual('FAILED', self.machine.state)
        self.assertEqual('lift_failed', self.machine.failure_type)

    def test_pick_execution_cannot_start_before_refined_pose_or_after_failure(self):
        self.assertFalse(self.machine.begin_pick_execution())
        self.navigate_and_stop()
        self.machine.observation_finished(True, '')
        self.machine.accept_refined_pose(7.0, 'aubo_i5_base_link')
        self.assertTrue(self.machine.begin_pick_execution())
        self.assertTrue(self.machine.pick_started)
        self.machine.fail('manipulation_failed', 'forced')
        self.assertFalse(self.machine.begin_pick_execution())


if __name__ == '__main__':
    unittest.main()
