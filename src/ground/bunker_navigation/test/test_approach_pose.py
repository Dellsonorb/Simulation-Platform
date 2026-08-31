#!/usr/bin/env python
from __future__ import division

import math
import unittest

from bunker_navigation.approach_pose import (ApproachParameters,
                                               candidate_bearing_is_supported,
                                               candidate_is_clear,
                                               costmap_is_ready,
                                               generate_candidates,
                                               normalize_angle,
                                               path_sweep_avoids_target)
from bunker_navigation.costmap_sweep import CostmapGrid


class ApproachPoseTest(unittest.TestCase):
    FOOTPRINT = ((-0.52, -0.39), (-0.52, 0.39),
                 (0.52, 0.39), (0.52, -0.39))

    @staticmethod
    def parameters(work_distance=0.82):
        return ApproachParameters(
            work_distance=work_distance,
            arm_offset_x=0.15,
            arm_offset_y=0.0,
            reach_min=0.75,
            reach_max=0.88,
            lateral_limit=0.12)

    @staticmethod
    def grid(obstacles=()):
        resolution = 0.02
        width = height = 400
        origin = -4.0
        data = [0] * (width * height)
        for x, y in obstacles:
            mx = int(math.floor((x - origin) / resolution))
            my = int(math.floor((y - origin) / resolution))
            data[my * width + mx] = 100
        return CostmapGrid(width, height, resolution, origin, origin, data)

    def assert_brick_in_workspace(self, candidate, brick):
        dx = brick[0] - candidate.x
        dy = brick[1] - candidate.y
        cosine = math.cos(candidate.yaw)
        sine = math.sin(candidate.yaw)
        base_x = cosine * dx + sine * dy
        base_y = -sine * dx + cosine * dy
        arm_x = base_x - self.parameters().arm_offset_x
        arm_y = base_y - self.parameters().arm_offset_y
        self.assertAlmostEqual(arm_x, 0.82, places=9)
        self.assertAlmostEqual(arm_y, 0.0, places=9)

    def test_primary_candidate_faces_brick_at_verified_arm_distance(self):
        brick = (2.0, 1.0, 0.7)
        candidates = generate_candidates(
            brick, current_pose=(0.0, 0.0, 0.0),
            parameters=self.parameters())
        primary = candidates[0]
        self.assertEqual(primary.identifier, 'direct')
        self.assertAlmostEqual(
            normalize_angle(primary.yaw - math.atan2(1.0, 2.0)), 0.0)
        self.assert_brick_in_workspace(primary, brick)

    def test_brick_orientation_provides_four_explainable_fallbacks(self):
        brick = (2.0, 0.0, math.radians(30.0))
        candidates = generate_candidates(
            brick, current_pose=(0.0, 0.0, 0.0),
            parameters=self.parameters())
        by_name = dict((item.identifier, item) for item in candidates)
        self.assertEqual(set(by_name),
                         set(('direct', 'brick_long_positive',
                              'brick_long_negative', 'brick_short_positive',
                              'brick_short_negative')))
        self.assertAlmostEqual(by_name['brick_long_positive'].yaw,
                               math.radians(30.0))
        self.assertAlmostEqual(abs(normalize_angle(
            by_name['brick_short_positive'].yaw - math.radians(120.0))), 0.0)
        for candidate in candidates:
            self.assert_brick_in_workspace(candidate, brick)

    def test_duplicate_axis_direction_is_removed_deterministically(self):
        candidates = generate_candidates(
            (2.0, 0.0, 0.0), current_pose=(0.0, 0.0, 0.0),
            parameters=self.parameters())
        self.assertEqual([item.identifier for item in candidates],
                         ['direct', 'brick_long_negative',
                          'brick_short_positive', 'brick_short_negative'])

    def test_invalid_work_distance_is_rejected_before_generation(self):
        with self.assertRaisesRegexp(ValueError, 'work_distance'):
            generate_candidates(
                (2.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                self.parameters(work_distance=0.90))

    def test_candidate_footprint_rejects_obstacle_and_unknown_cells(self):
        candidate = generate_candidates(
            (2.0, 0.0, 0.0), (0.0, 0.0, 0.0),
            self.parameters())[0]
        clear, reason = candidate_is_clear(
            self.grid(), self.FOOTPRINT, candidate, lethal_threshold=100)
        self.assertTrue(clear)
        self.assertEqual(reason, 'CLEAR')

        clear, reason = candidate_is_clear(
            self.grid(((candidate.x, candidate.y),)),
            self.FOOTPRINT, candidate, lethal_threshold=100)
        self.assertFalse(clear)
        self.assertEqual(reason, 'COLLISION')

        unknown = self.grid()
        mx = int(math.floor((candidate.x - unknown.origin_x) /
                            unknown.resolution))
        my = int(math.floor((candidate.y - unknown.origin_y) /
                            unknown.resolution))
        unknown.data[my * unknown.width + mx] = -1
        clear, reason = candidate_is_clear(
            unknown, self.FOOTPRINT, candidate, lethal_threshold=100)
        self.assertFalse(clear)
        self.assertEqual(reason, 'UNKNOWN')

    def test_candidate_avoids_dwa_lateral_band_but_keeps_rear_gate(self):
        parameters = self.parameters()
        side = generate_candidates(
            (0.0, 2.0, math.pi / 2.0), (0.0, 0.0, 0.0),
            parameters)[0]
        forward = generate_candidates(
            (2.0, 0.5, 0.0), (0.0, 0.0, 0.0), parameters)[0]
        rear = generate_candidates(
            (-2.0, 0.0, 0.0), (0.0, 0.0, 0.0), parameters)[0]
        diagonal = generate_candidates(
            (2.0, 2.0, 0.0), (0.0, 0.0, 0.0), parameters)[0]
        lower = math.radians(30.0)
        gate_entry = math.radians(100.0)
        self.assertFalse(candidate_bearing_is_supported(
            side, (0.0, 0.0, 0.0), lower, gate_entry))
        self.assertTrue(candidate_bearing_is_supported(
            forward, (0.0, 0.0, 0.0), lower, gate_entry))
        self.assertFalse(candidate_bearing_is_supported(
            diagonal, (0.0, 0.0, 0.0), lower, gate_entry))
        self.assertTrue(candidate_bearing_is_supported(
            rear, (0.0, 0.0, 0.0), lower, gate_entry))

    def test_short_side_plan_sweeping_through_known_brick_is_rejected(self):
        brick = (1.48, 0.08, 0.70)
        short_side = (
            (0.0, 0.0, -0.27),
            (1.00, -0.27, -0.27),
            (2.007, -0.549, 2.27),
        )
        clear, reason, checked = path_sweep_avoids_target(
            short_side, self.FOOTPRINT, brick,
            brick_length=0.240, brick_width=0.115,
            target_padding=0.03, linear_step=0.025,
            angular_step=0.025)
        self.assertFalse(clear)
        self.assertEqual('TARGET_SWEEP_COLLISION', reason)
        self.assertGreater(checked, 2)

    def test_long_side_plan_with_safe_standoff_is_accepted(self):
        brick = (1.48, 0.08, 0.70)
        long_side = (
            (0.0, 0.0, -0.48),
            (0.45, -0.23, -0.48),
            (0.853, -0.448, 0.70),
        )
        clear, reason, checked = path_sweep_avoids_target(
            long_side, self.FOOTPRINT, brick,
            brick_length=0.240, brick_width=0.115,
            target_padding=0.03, linear_step=0.025,
            angular_step=0.025)
        self.assertTrue(clear)
        self.assertEqual('CLEAR', reason)
        self.assertGreater(checked, len(long_side))

    def test_sparse_plan_is_interpolated_before_target_check(self):
        brick = (1.0, 0.0, 0.0)
        # Endpoints are clear; only the unsampled midpoint overlaps the brick.
        clear, reason, checked = path_sweep_avoids_target(
            ((0.0, 0.0, 0.0), (2.0, 0.0, 0.0)),
            ((-0.20, -0.20), (-0.20, 0.20),
             (0.20, 0.20), (0.20, -0.20)),
            brick, brick_length=0.24, brick_width=0.115,
            target_padding=0.03, linear_step=0.05,
            angular_step=0.05)
        self.assertFalse(clear)
        self.assertEqual('TARGET_SWEEP_COLLISION', reason)
        self.assertGreater(checked, 2)

    def test_target_sweep_rejects_non_finite_or_invalid_geometry(self):
        with self.assertRaisesRegexp(ValueError, 'finite'):
            path_sweep_avoids_target(
                ((0.0, 0.0, 0.0), (float('nan'), 0.0, 0.0)),
                self.FOOTPRINT, (1.0, 0.0, 0.0), 0.24, 0.115,
                0.03, 0.05, 0.05)
        with self.assertRaisesRegexp(ValueError, 'positive'):
            path_sweep_avoids_target(
                ((0.0, 0.0, 0.0),), self.FOOTPRINT,
                (1.0, 0.0, 0.0), 0.0, 0.115, 0.03, 0.05, 0.05)

    def test_costmap_readiness_requires_post_input_settle_update(self):
        self.assertFalse(costmap_is_ready(
            costmap_received_at=10.1, now=10.2, input_received_at=10.0,
            max_age=1.0, settle_duration=0.8))
        self.assertFalse(costmap_is_ready(
            costmap_received_at=10.7, now=10.9, input_received_at=10.0,
            max_age=1.0, settle_duration=0.8))
        self.assertTrue(costmap_is_ready(
            costmap_received_at=10.85, now=10.9, input_received_at=10.0,
            max_age=1.0, settle_duration=0.8))

    def test_costmap_readiness_rejects_stale_or_invalid_timing(self):
        self.assertFalse(costmap_is_ready(
            costmap_received_at=10.9, now=12.0, input_received_at=10.0,
            max_age=1.0, settle_duration=0.8))
        with self.assertRaises(ValueError):
            costmap_is_ready(1.0, 1.0, 1.0, -1.0, 0.8)


if __name__ == '__main__':
    unittest.main()
