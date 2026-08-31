#!/usr/bin/env python
from __future__ import division

import math
import unittest

from brick_rgbd_perception.validation import (
    observation_joint_error, pose_errors, representative_pose,
    select_stamped_samples, summarize_scenarios, validate_scenarios)


class ValidationTest(unittest.TestCase):
    def test_pose_errors_use_pi_equivalent_yaw(self):
        errors = pose_errors(
            (1.003, 1.004, 0.497), 0.2 + math.pi,
            (1.0, 1.0, 0.5), 0.2)

        self.assertAlmostEqual(0.005, errors['xy'], places=12)
        self.assertAlmostEqual(0.003, errors['z'], places=12)
        self.assertAlmostEqual(math.sqrt(0.000034), errors['position'],
                               places=12)
        self.assertAlmostEqual(0.0, errors['yaw'], places=12)

    def test_summary_counts_rejections_and_uses_one_value_per_pose(self):
        results = [
            {'index': 0, 'category': 'center', 'expect': 'success',
             'outcome': 'success', 'errors': {
                'xy': 0.001, 'z': 0.002, 'position': 0.003, 'yaw': 0.01}},
            {'index': 1, 'category': 'center', 'expect': 'success',
             'outcome': 'rejected', 'errors': None},
            {'index': 2, 'category': 'image_boundary', 'expect': 'success',
             'outcome': 'success', 'errors': {
                'xy': 0.003, 'z': 0.004, 'position': 0.005, 'yaw': 0.03}},
            {'index': 3, 'category': 'expected_rejection',
             'expect': 'rejected', 'outcome': 'rejected', 'errors': None},
        ]

        summary = summarize_scenarios(results)

        self.assertEqual(4, summary['total'])
        self.assertEqual(2, summary['successes'])
        self.assertEqual(2, summary['rejections'])
        self.assertAlmostEqual(0.5, summary['success_rate'])
        self.assertAlmostEqual(0.5, summary['rejection_rate'])
        self.assertAlmostEqual(0.002, summary['metrics']['xy']['mean'])
        self.assertAlmostEqual(0.002, summary['metrics']['xy']['median'])
        self.assertAlmostEqual(0.003, summary['metrics']['xy']['max'])
        self.assertEqual(2, summary['metrics']['position']['worst_index'])
        self.assertEqual(1, summary['categories']['center']['successes'])
        self.assertEqual(1, summary['categories']['center']['rejections'])
        self.assertAlmostEqual(0.75, summary['expected_match_rate'])

    def test_representative_pose_is_stable_across_axis_angle_seam(self):
        samples = [
            ((0.601, -0.002, -0.456), math.radians(89.8)),
            ((0.599, 0.002, -0.457), math.radians(-89.9)),
            ((0.600, 0.000, -0.455), math.radians(89.9)),
        ]

        position, yaw, spread = representative_pose(samples)

        self.assertAlmostEqual(0.600, position[0], places=12)
        self.assertAlmostEqual(0.000, position[1], places=12)
        self.assertLess(abs(position[2] + 0.456), 1e-12)
        self.assertLess(abs(abs(yaw) - math.pi / 2.0), math.radians(0.2))
        self.assertLess(spread['position_max'], 0.003)
        self.assertLess(spread['yaw_max'], math.radians(0.3))

    def test_stamped_samples_must_be_unique_and_span_time(self):
        samples = [(1.0, 'a'), (1.0, 'duplicate'), (1.1, 'b'),
                   (1.2, 'c'), (1.4, 'd')]

        selected = select_stamped_samples(samples, 4, 0.35)

        self.assertEqual([(1.0, 'a'), (1.1, 'b'), (1.2, 'c'),
                          (1.4, 'd')], selected)
        self.assertIsNone(select_stamped_samples(samples, 4, 0.5))

        late_gap = [(0.00, 'a'), (0.01, 'b'), (0.02, 'c'), (0.03, 'd'),
                    (0.04, 'e'), (0.05, 'f'), (0.06, 'g'), (0.31, 'h')]
        selected = select_stamped_samples(late_gap, 7, 0.30)
        self.assertEqual(0.01, selected[0][0])
        self.assertEqual(0.31, selected[-1][0])

    def test_observation_joint_error_requires_every_expected_joint(self):
        expected = {'joint_a': 0.2, 'joint_b': -0.4}
        self.assertAlmostEqual(0.01, observation_joint_error(
            {'joint_a': 0.19, 'joint_b': -0.4}, expected))
        with self.assertRaises(ValueError):
            observation_joint_error({'joint_a': 0.2}, expected)

    def test_scenario_validation_requires_stratified_unique_20_to_30(self):
        scenarios = []
        categories = ('center', 'workspace_edge', 'image_boundary',
                      'expected_rejection')
        for index in range(25):
            scenarios.append({
                'id': 'pose_%02d' % (index + 1),
                'category': categories[index % len(categories)],
                'x': 0.70 + 0.002 * index,
                'y': -0.05 + 0.004 * index,
                'yaw': -1.2 + 0.1 * index,
                'expect': ('rejected' if categories[index % 4] ==
                           'expected_rejection' else 'success'),
            })

        validate_scenarios(scenarios)

        with self.assertRaises(ValueError):
            validate_scenarios(scenarios[:19])
        duplicate = list(scenarios)
        duplicate[-1] = dict(duplicate[0])
        with self.assertRaises(ValueError):
            validate_scenarios(duplicate)
        unknown_category = list(scenarios)
        unknown_category[-1] = dict(unknown_category[-1])
        unknown_category[-1]['category'] = 'unclassified'
        with self.assertRaises(ValueError):
            validate_scenarios(unknown_category)
        non_finite = list(scenarios)
        non_finite[-1] = dict(non_finite[-1])
        non_finite[-1]['x'] = float('nan')
        with self.assertRaises(ValueError):
            validate_scenarios(non_finite)


if __name__ == '__main__':
    unittest.main()
