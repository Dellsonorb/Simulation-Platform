#!/usr/bin/env python
from __future__ import division

import unittest

from brick_visual_pick.trial_results import (PickProgress, is_lifted,
                                             summarize_trials)


class TrialResultsTest(unittest.TestCase):
    def test_stage_logs_set_monotonic_success_flags(self):
        progress = PickProgress()
        progress.handle_status('POSE_PUBLISHED')
        progress.handle_log('[MOVE_PREGRASP] executing planned trajectory')
        progress.handle_log('[VERIFY_GRASP] brick attached (simulation-only)')
        progress.handle_log('[SUCCESS] brick pick and lift completed')
        self.assertTrue(progress.perception_success)
        self.assertTrue(progress.planning_success)
        self.assertTrue(progress.grasp_success)
        self.assertTrue(progress.manipulation_success)

    def test_rejection_is_explicit_and_does_not_count_as_perception(self):
        progress = PickProgress()
        progress.handle_status('PERCEPTION_REJECTED:timeout_no_stable_pose')
        self.assertFalse(progress.perception_success)
        self.assertEqual(progress.failure_type, 'perception_rejected')
        self.assertIn('timeout_no_stable_pose', progress.failure_detail)
        self.assertFalse(progress.execution_started)

    def test_lift_requires_physical_height_change(self):
        self.assertTrue(is_lifted(0.0265, 0.1720, 0.10))
        self.assertFalse(is_lifted(0.0265, 0.0800, 0.10))

    def test_summary_uses_all_valid_trials_as_denominator(self):
        trials = [
            {'perception_success': True, 'planning_success': True,
             'grasp_success': True, 'end_to_end_success': True},
            {'perception_success': True, 'planning_success': False,
             'grasp_success': False, 'end_to_end_success': False},
            {'perception_success': False, 'planning_success': False,
             'grasp_success': False, 'end_to_end_success': False},
        ]
        result = summarize_trials(trials)
        self.assertEqual(result['trial_count'], 3)
        self.assertAlmostEqual(result['perception_rate'], 2.0 / 3.0)
        self.assertAlmostEqual(result['planning_rate'], 1.0 / 3.0)
        self.assertAlmostEqual(result['grasp_rate'], 1.0 / 3.0)
        self.assertAlmostEqual(result['end_to_end_rate'], 1.0 / 3.0)
        self.assertEqual(result['failure_counts'], {
            'planning_failed': 1, 'perception_failed': 1})


if __name__ == '__main__':
    unittest.main()
