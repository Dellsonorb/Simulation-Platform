from __future__ import division

import unittest

from ground_pick_orchestrator.mission_metrics import (acceptance_passed,
                                                       summarize_trials)


class MissionMetricsTest(unittest.TestCase):
    def successful(self, identifier, duration=10.0):
        return {
            'scenario_id': identifier,
            'expect_success': True,
            'navigation_success': True,
            'perception_success': True,
            'planning_success': True,
            'grasp_success': True,
            'end_to_end_success': True,
            'failure_type': '',
            'duration': duration,
            'fresh_pose_after_stop': True,
            'safe_stop': True,
            'container_exit_code': 0,
        }

    def test_success_rates_and_slowest_case(self):
        trials = [self.successful('a', 8.0), self.successful('b', 12.0)]
        summary = summarize_trials(trials)
        self.assertEqual(2, summary['valid_trial_count'])
        self.assertEqual(1.0, summary['navigation_rate'])
        self.assertEqual(1.0, summary['refine_perception_rate'])
        self.assertEqual(1.0, summary['planning_rate'])
        self.assertEqual(1.0, summary['grasp_rate'])
        self.assertEqual(1.0, summary['end_to_end_rate'])
        self.assertEqual('b', summary['slowest_case']['scenario_id'])

    def test_failure_categories_are_counted(self):
        trials = [self.successful('good')]
        failed = self.successful('planning')
        failed.update(planning_success=False, grasp_success=False,
                      end_to_end_success=False,
                      failure_type='planning_failed')
        trials.append(failed)
        summary = summarize_trials(trials)
        self.assertEqual(0.5, summary['planning_rate'])
        self.assertEqual({'planning_failed': 1},
                         summary['failure_counts'])

    def test_controls_require_expected_safe_terminal_failure(self):
        valid = [self.successful(str(index)) for index in range(10)]
        controls = [
            {'scenario_id': 'navigation_reject', 'expect_success': False,
             'expected_failure': 'navigation_failed',
             'failure_type': 'navigation_failed', 'safe_stop': True,
             'pick_started': False, 'container_exit_code': 0},
            {'scenario_id': 'perception_reject', 'expect_success': False,
             'expected_failure': 'perception_rejected',
             'failure_type': 'perception_rejected', 'safe_stop': True,
             'pick_started': False, 'container_exit_code': 0},
            {'scenario_id': 'planning_reject', 'expect_success': False,
             'expected_failure': 'planning_failed',
             'failure_type': 'planning_failed', 'safe_stop': True,
             'pick_started': True, 'container_exit_code': 0},
        ]
        summary = summarize_trials(valid + controls)
        self.assertEqual(3, summary['control_count'])
        self.assertEqual(3, summary['safe_control_count'])
        self.assertTrue(acceptance_passed(summary, minimum_valid=10,
                                          expected_controls=3))

        controls[0]['pick_started'] = True
        summary = summarize_trials(valid + controls)
        self.assertFalse(acceptance_passed(summary, 10, 3))

    def test_acceptance_cannot_pass_with_fewer_than_ten_valid_trials(self):
        summary = summarize_trials(
            [self.successful(str(index)) for index in range(9)])
        self.assertFalse(acceptance_passed(summary, minimum_valid=10,
                                           expected_controls=0))

    def test_nonzero_container_exit_never_passes(self):
        trials = [self.successful(str(index)) for index in range(10)]
        trials[5]['container_exit_code'] = 137
        summary = summarize_trials(trials)
        self.assertFalse(acceptance_passed(summary, 10, 0))


if __name__ == '__main__':
    unittest.main()
