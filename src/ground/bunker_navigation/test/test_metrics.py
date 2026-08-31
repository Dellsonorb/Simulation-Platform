#!/usr/bin/env python
from __future__ import division

import math
import unittest

from bunker_navigation.metrics import (evaluate_navigation, pose_error,
                                       heading_alignment_evidence,
                                       relative_transform_error,
                                       summarize_trials)


class MetricsTest(unittest.TestCase):
    def test_pose_error_wraps_yaw(self):
        result = pose_error((1.01, 2.02, -math.pi + 0.01),
                            (1.0, 2.0, math.pi - 0.01))
        self.assertAlmostEqual(result['xy'], math.sqrt(0.0005))
        self.assertAlmostEqual(result['yaw'], 0.02)

    def test_success_requires_action_accuracy_stop_and_tf_integrity(self):
        result = evaluate_navigation(
            action_succeeded=True, xy_error=0.025, yaw_error=0.02,
            linear_speed=0.002, angular_speed=0.003,
            tf_preserved=True, xy_tolerance=0.05, yaw_tolerance=0.05,
            stop_linear_tolerance=0.01, stop_angular_tolerance=0.01)
        self.assertTrue(result['success'])
        result = evaluate_navigation(
            action_succeeded=True, xy_error=0.025, yaw_error=0.02,
            linear_speed=0.02, angular_speed=0.003,
            tf_preserved=True, xy_tolerance=0.05, yaw_tolerance=0.05,
            stop_linear_tolerance=0.01, stop_angular_tolerance=0.01)
        self.assertFalse(result['success'])
        self.assertEqual(result['failure_type'], 'not_stopped')

    def test_summary_uses_all_trials_and_identifies_worst(self):
        trials = [
            {'scenario_id': 'good', 'success': True, 'xy_error': 0.01,
             'yaw_error': 0.02, 'duration': 8.0, 'failure_type': ''},
            {'scenario_id': 'bad', 'success': False, 'xy_error': 0.08,
             'yaw_error': 0.04, 'duration': 12.0,
             'failure_type': 'xy_error'},
        ]
        result = summarize_trials(trials)
        self.assertEqual(result['trial_count'], 2)
        self.assertEqual(result['success_count'], 1)
        self.assertEqual(result['success_rate'], 0.5)
        self.assertEqual(result['failure_counts'], {'xy_error': 1})
        self.assertEqual(result['worst_xy_case']['scenario_id'], 'bad')
        self.assertEqual(result['slowest_case']['scenario_id'], 'bad')

    def test_relative_transform_error_handles_quaternion_sign(self):
        reference = ((0.1, -0.2, 0.3), (0.0, 0.0, 0.0, 1.0))
        current = ((0.101, -0.2, 0.3), (0.0, 0.0, 0.0, -1.0))
        result = relative_transform_error(reference, current)
        self.assertAlmostEqual(result['translation'], 0.001)
        self.assertAlmostEqual(result['rotation'], 0.0)

    def test_heading_evidence_excludes_first_command_after_release(self):
        samples = [
            {'bearing': -2.8, 'linear': 0.0, 'angular': 0.0},
            {'bearing': -2.8, 'linear': 0.0, 'angular': -0.8},
            {'bearing': -0.7, 'linear': 0.0, 'angular': -0.8},
            # State and command topics may arrive in either order. This is the
            # first DWA command after the actual 30-degree release.
            {'bearing': -0.50, 'linear': 0.04, 'angular': -0.4},
        ]
        evidence = heading_alignment_evidence(
            samples, initial_bearing=-2.8,
            release_bearing=math.radians(30.0))
        self.assertEqual(evidence['sample_count'], 3)
        self.assertEqual(evidence['max_linear'], 0.0)
        self.assertEqual(evidence['first_angular'], -0.8)
        self.assertTrue(evidence['correct_turn_direction'])


if __name__ == '__main__':
    unittest.main()
