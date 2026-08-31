#!/usr/bin/env python
from __future__ import division

import math
import unittest

from brick_visual_pick.quality_gate import evaluate_pose_samples


def sample(stamp, x=0.65, y=0.0, z=0.0265, yaw=0.20,
           frame='aubo_i5_base_link'):
    return {
        'stamp': stamp, 'frame_id': frame,
        'x': x, 'y': y, 'z': z, 'yaw': yaw,
    }


class QualityGateTest(unittest.TestCase):
    def test_accepts_stable_unique_samples(self):
        samples = [sample(1.0 + 0.1 * index,
                          x=0.65 + 0.0002 * index,
                          y=-0.02 - 0.0001 * index,
                          yaw=0.20 + 0.001 * index)
                   for index in range(5)]
        result = evaluate_pose_samples(samples, 5, 0.30, 0.003, 0.02,
                                       'aubo_i5_base_link')
        self.assertTrue(result['accepted'])
        self.assertEqual(result['reason'], 'accepted')
        self.assertAlmostEqual(result['pose']['x'], 0.6504, places=6)
        self.assertAlmostEqual(result['pose']['y'], -0.0202, places=6)

    def test_yaw_is_averaged_modulo_pi(self):
        samples = [sample(1.0 + 0.1 * index,
                          yaw=(math.pi / 2.0 - 0.004
                               if index % 2 == 0
                               else -math.pi / 2.0 + 0.004))
                   for index in range(5)]
        result = evaluate_pose_samples(samples, 5, 0.30, 0.003, 0.02,
                                       'aubo_i5_base_link')
        self.assertTrue(result['accepted'])
        self.assertLess(abs(abs(result['pose']['yaw']) - math.pi / 2.0),
                        0.005)

    def test_duplicate_stamps_do_not_satisfy_sample_count(self):
        samples = [sample(1.0), sample(1.0), sample(1.1), sample(1.2),
                   sample(1.3)]
        result = evaluate_pose_samples(samples, 5, 0.20, 0.003, 0.02,
                                       'aubo_i5_base_link')
        self.assertFalse(result['accepted'])
        self.assertEqual(result['reason'], 'insufficient_unique_samples')

    def test_rejects_wrong_frame(self):
        samples = [sample(1.0 + 0.1 * index, frame='world')
                   for index in range(5)]
        result = evaluate_pose_samples(samples, 5, 0.30, 0.003, 0.02,
                                       'aubo_i5_base_link')
        self.assertFalse(result['accepted'])
        self.assertEqual(result['reason'], 'wrong_frame')

    def test_rejects_unstable_position(self):
        samples = [sample(1.0 + 0.1 * index,
                          x=0.65 + (0.010 if index == 4 else 0.0))
                   for index in range(5)]
        result = evaluate_pose_samples(samples, 5, 0.30, 0.003, 0.02,
                                       'aubo_i5_base_link')
        self.assertFalse(result['accepted'])
        self.assertEqual(result['reason'], 'position_unstable')

    def test_rejects_short_time_span(self):
        samples = [sample(1.0 + 0.01 * index) for index in range(5)]
        result = evaluate_pose_samples(samples, 5, 0.30, 0.003, 0.02,
                                       'aubo_i5_base_link')
        self.assertFalse(result['accepted'])
        self.assertEqual(result['reason'], 'insufficient_time_span')

    def test_rejects_non_finite_estimate(self):
        samples = [sample(1.0 + 0.1 * index) for index in range(6)]
        samples[2]['x'] = float('nan')
        result = evaluate_pose_samples(samples, 6, 0.30, 0.003, 0.02,
                                       'aubo_i5_base_link')
        self.assertFalse(result['accepted'])
        self.assertEqual(result['reason'], 'non_finite_sample')


if __name__ == '__main__':
    unittest.main()
