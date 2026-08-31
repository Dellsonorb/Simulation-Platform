from __future__ import division

import math
import unittest

from ground_pick_orchestrator.safety_guards import (motion_is_stopped,
                                                     joint_window_is_settled,
                                                     observation_result_is_safe,
                                                     refined_pose_is_valid,
                                                     terminal_stop_is_safe)


class SafetyGuardsTest(unittest.TestCase):
    def test_observation_settle_uses_position_window_not_noisy_velocity(self):
        targets = {'j1': 1.0, 'j2': -0.5}
        samples = [
            (0.0, {'j1': 1.002, 'j2': -0.501}),
            (0.3, {'j1': 0.999, 'j2': -0.500}),
            (0.6, {'j1': 1.001, 'j2': -0.499}),
        ]
        self.assertTrue(joint_window_is_settled(
            samples, targets, 0.08, 0.005, 0.5))
        samples[-1][1]['j1'] = 1.010
        self.assertFalse(joint_window_is_settled(
            samples, targets, 0.08, 0.005, 0.5))

    def test_observation_single_sample_race_requires_gate_verification(self):
        self.assertTrue(observation_result_is_safe(0, False))
        self.assertTrue(observation_result_is_safe(6, True))
        self.assertFalse(observation_result_is_safe(6, False))
        self.assertFalse(observation_result_is_safe(5, True))

    def test_motion_requires_both_odom_and_command_to_be_stopped(self):
        self.assertTrue(motion_is_stopped(
            (0.001, -0.001, 0.002), (0.0, 0.0), 0.01, 0.02))
        self.assertFalse(motion_is_stopped(
            (0.02, 0.0, 0.0), (0.0, 0.0), 0.01, 0.02))
        self.assertFalse(motion_is_stopped(
            (0.0, 0.0, 0.0), (0.03, 0.0), 0.01, 0.02))
        self.assertFalse(motion_is_stopped(
            None, (0.0, 0.0), 0.01, 0.02))
        self.assertFalse(motion_is_stopped(
            (0.0, 0.0, 0.0), None, 0.01, 0.02))

    def test_refined_pose_requires_finite_values_and_unit_quaternion(self):
        valid = (7.0, 0.82, 0.0, 0.0265, 0.0, 0.0, 0.0, 1.0)
        self.assertTrue(refined_pose_is_valid(valid))
        for index in range(len(valid)):
            values = list(valid)
            values[index] = float('nan')
            self.assertFalse(refined_pose_is_valid(values))
        invalid = list(valid)
        invalid[-1] = 0.0
        self.assertFalse(refined_pose_is_valid(invalid))
        invalid[-1] = 1.02
        self.assertFalse(refined_pose_is_valid(invalid))

    def test_thresholds_are_inclusive(self):
        self.assertTrue(motion_is_stopped(
            (0.01, -0.01, 0.02), (0.01, -0.02), 0.01, 0.02))

    def test_terminal_stop_requires_fresh_zero_command_and_stationary_model(self):
        first = (1.0, 2.0, math.pi - 0.001, 0.001, -0.001, 0.002)
        second = (1.001, 1.999, -math.pi + 0.001, 0.001, 0.0, 0.001)
        self.assertTrue(terminal_stop_is_safe(
            (0.0, 0.0), True, first, second, 0.5, 0.01, 0.02))
        self.assertFalse(terminal_stop_is_safe(
            (0.0, 0.0), False, first, second, 0.5, 0.01, 0.02))
        self.assertFalse(terminal_stop_is_safe(
            None, True, first, second, 0.5, 0.01, 0.02))
        self.assertFalse(terminal_stop_is_safe(
            (0.0, 0.0), True, first, second, 0.1, 0.01, 0.02))

    def test_terminal_stop_rejects_motion_and_non_finite_samples(self):
        stationary = (1.0, 2.0, 0.1, 0.0, 0.0, 0.0)
        displaced = (1.02, 2.0, 0.1, 0.0, 0.0, 0.0)
        spinning = (1.0, 2.0, 0.1, 0.0, 0.0, 0.03)
        invalid = (1.0, 2.0, float('nan'), 0.0, 0.0, 0.0)
        self.assertFalse(terminal_stop_is_safe(
            (0.0, 0.0), True, stationary, displaced, 0.5, 0.01, 0.02))
        self.assertFalse(terminal_stop_is_safe(
            (0.0, 0.0), True, spinning, stationary, 0.5, 0.01, 0.02))
        self.assertFalse(terminal_stop_is_safe(
            (0.0, 0.0), True, invalid, stationary, 0.5, 0.01, 0.02))


if __name__ == '__main__':
    unittest.main()
