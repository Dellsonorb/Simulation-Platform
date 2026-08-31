#!/usr/bin/env python
from __future__ import division

import math
import unittest

from bunker_navigation.approach_metrics import (relative_brick_metrics,
                                                  summarize_approach_trials)


class ApproachMetricsTest(unittest.TestCase):
    def test_relative_metrics_use_bunker_and_arm_mount_geometry(self):
        result = relative_brick_metrics(
            base_pose=(1.03, 0.0, 0.0), brick_pose=(2.0, 0.0, 0.4),
            arm_offset_x=0.15, arm_offset_y=0.0,
            work_distance=0.82)
        self.assertAlmostEqual(result['center_distance'], 0.97)
        self.assertAlmostEqual(result['distance_error'], 0.0)
        self.assertAlmostEqual(result['facing_error'], 0.0)
        self.assertAlmostEqual(result['arm_forward'], 0.82)
        self.assertAlmostEqual(result['arm_lateral'], 0.0)
        self.assertAlmostEqual(result['arm_workspace_error'], 0.0)

    def test_relative_metrics_handle_rotated_vehicle(self):
        result = relative_brick_metrics(
            base_pose=(2.0, 1.03, -math.pi / 2.0),
            brick_pose=(2.0, 0.0, 0.0),
            arm_offset_x=0.15, arm_offset_y=0.0,
            work_distance=0.88)
        self.assertAlmostEqual(result['center_distance'], 1.03)
        self.assertAlmostEqual(result['facing_error'], 0.0)
        self.assertAlmostEqual(result['arm_forward'], 0.88)
        self.assertAlmostEqual(result['arm_lateral'], 0.0)

    def test_summary_separates_generation_navigation_and_rejection(self):
        trials = [
            {'expect_generation': True, 'generation_success': True,
             'navigation_success': True, 'distance_error': 0.01,
             'facing_error': 0.02, 'duration': 4.0},
            {'expect_generation': True, 'generation_success': True,
             'navigation_success': False, 'distance_error': 0.20,
             'facing_error': 0.30, 'duration': 7.0},
            {'expect_generation': False, 'generation_success': False,
             'navigation_success': False, 'distance_error': None,
             'facing_error': None, 'duration': 2.0},
        ]
        result = summarize_approach_trials(trials)
        self.assertEqual(result['trial_count'], 3)
        self.assertEqual(result['valid_trial_count'], 2)
        self.assertEqual(result['generation_success_count'], 2)
        self.assertEqual(result['navigation_success_count'], 1)
        self.assertAlmostEqual(result['generation_success_rate'], 1.0)
        self.assertAlmostEqual(result['navigation_success_rate'], 0.5)
        self.assertEqual(result['safe_rejection_count'], 1)
        self.assertAlmostEqual(result['distance_error_max'], 0.20)
        self.assertAlmostEqual(result['facing_error_max'], 0.30)


if __name__ == '__main__':
    unittest.main()
