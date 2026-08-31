#!/usr/bin/env python
from __future__ import division

import unittest

from brick_visual_pick.matrix_report import (acceptance_passed,
                                             build_matrix_summary)


class MatrixReportTest(unittest.TestCase):
    def test_identifies_worst_pose_and_slowest_case(self):
        trials = [
            {'scenario_id': 'fast', 'perception_success': True,
             'planning_success': True, 'grasp_success': True,
             'end_to_end_success': True, 'wall_duration': 20.0,
             'pose_error': {'position': 0.001, 'xy': 0.001,
                            'z': 0.0, 'yaw': 0.01}},
            {'scenario_id': 'slow_worst', 'perception_success': True,
             'planning_success': True, 'grasp_success': True,
             'end_to_end_success': True, 'wall_duration': 25.0,
             'pose_error': {'position': 0.004, 'xy': 0.003,
                            'z': 0.002, 'yaw': 0.04}},
        ]
        result = build_matrix_summary(trials)
        self.assertEqual(result['worst_pose_case']['scenario_id'],
                         'slow_worst')
        self.assertEqual(result['slowest_case']['scenario_id'],
                         'slow_worst')
        self.assertEqual(result['rates']['end_to_end_rate'], 1.0)

    def test_failed_case_is_reported_separately(self):
        trials = [
            {'scenario_id': 'failed', 'perception_success': True,
             'planning_success': False, 'grasp_success': False,
             'end_to_end_success': False, 'wall_duration': 18.0,
             'failure_type': 'planning_failed', 'failure_detail': 'no plan',
             'pose_error': {'position': 0.001, 'xy': 0.001,
                            'z': 0.0, 'yaw': 0.01}},
        ]
        result = build_matrix_summary(trials)
        self.assertEqual(result['failed_cases'][0]['scenario_id'], 'failed')
        self.assertEqual(result['rates']['failure_counts'],
                         {'planning_failed': 1})
        self.assertFalse(acceptance_passed(result, 1,
                                          {'control_safe': True}))

    def test_acceptance_requires_at_least_twenty_all_success_and_safe_control(self):
        trial = {
            'scenario_id': 'ok', 'perception_success': True,
            'planning_success': True, 'grasp_success': True,
            'end_to_end_success': True, 'wall_duration': 20.0,
            'pose_error': None}
        trials = [dict(trial, scenario_id='ok_{}'.format(index))
                  for index in range(20)]
        summary = build_matrix_summary(trials)
        self.assertTrue(acceptance_passed(summary, 20,
                                         {'control_safe': True}))
        short_summary = build_matrix_summary([
            {'scenario_id': 'ok', 'perception_success': True,
             'planning_success': True, 'grasp_success': True,
             'end_to_end_success': True, 'wall_duration': 20.0,
             'pose_error': None},
        ])
        self.assertFalse(acceptance_passed(short_summary, 1,
                                          {'control_safe': True}))
        self.assertFalse(acceptance_passed(summary, 20,
                                          {'control_safe': False}))


if __name__ == '__main__':
    unittest.main()
