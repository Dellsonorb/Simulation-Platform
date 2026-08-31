#!/usr/bin/env python3

import math
import unittest
import warnings

import numpy as np

from brick_aerial_perception.geometry import (
    backproject_mask,
    estimate_brick_pose,
    register_depth_to_color,
    yaw_error_mod_pi,
)


class DepthRegistrationTest(unittest.TestCase):
    def test_registers_depth_with_different_color_intrinsics(self):
        depth = np.zeros((3, 3), dtype=np.float32)
        depth[1, 1] = 2.0
        depth[1, 2] = 2.0
        depth_k = np.array([[100.0, 0.0, 1.0], [0.0, 100.0, 1.0], [0.0, 0.0, 1.0]])
        color_k = np.array([[200.0, 0.0, 2.0], [0.0, 200.0, 2.0], [0.0, 0.0, 1.0]])

        aligned = register_depth_to_color(depth, depth_k, color_k, (5, 5))

        self.assertAlmostEqual(float(aligned[2, 2]), 2.0)
        self.assertAlmostEqual(float(aligned[2, 4]), 2.0)
        self.assertTrue(np.isnan(aligned[1, 1]))

    def test_registration_uses_nearest_depth_for_pixel_collision(self):
        depth = np.array([[2.0, 1.0]], dtype=np.float32)
        depth_k = np.array([[1000.0, 0.0, 0.5], [0.0, 1000.0, 0.0], [0.0, 0.0, 1.0]])
        color_k = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])

        aligned = register_depth_to_color(depth, depth_k, color_k, (1, 1))

        self.assertAlmostEqual(float(aligned[0, 0]), 1.0)


class BackprojectionTest(unittest.TestCase):
    def test_filters_invalid_and_out_of_range_depth(self):
        mask = np.ones((2, 3), dtype=np.uint8)
        depth = np.array([[np.nan, 0.0, 1.0], [np.inf, 5.0, 2.0]], dtype=np.float32)
        camera_k = np.array([[100.0, 0.0, 1.0], [0.0, 100.0, 0.5], [0.0, 0.0, 1.0]])

        points = backproject_mask(mask, depth, camera_k, min_depth=0.2, max_depth=3.0)

        self.assertEqual(points.shape, (2, 3))
        self.assertTrue(np.isfinite(points).all())
        np.testing.assert_allclose(points[:, 2], [1.0, 2.0])

    def test_invalid_depth_filter_does_not_emit_numpy_runtime_warnings(self):
        depth = np.array([[np.nan, np.inf, 1.0]], dtype=np.float32)
        camera_k = np.array([[100.0, 0.0, 1.0], [0.0, 100.0, 0.0],
                             [0.0, 0.0, 1.0]])
        with warnings.catch_warnings():
            warnings.simplefilter("error", RuntimeWarning)
            points = backproject_mask(np.ones_like(depth), depth, camera_k)
        self.assertEqual(points.shape, (1, 3))


class BrickPoseTest(unittest.TestCase):
    def test_estimates_center_and_long_axis_modulo_pi(self):
        center_xy = np.array([1.2, -0.4])
        yaw = math.radians(30.0)
        length_axis = np.array([math.cos(yaw), math.sin(yaw)])
        width_axis = np.array([-math.sin(yaw), math.cos(yaw)])
        points = []
        for along in np.linspace(-0.10, 0.10, 21):
            for across in np.linspace(-0.05, 0.05, 11):
                xy = center_xy + along * length_axis + across * width_axis
                points.append([xy[0], xy[1], 0.1265])

        pose = estimate_brick_pose(np.asarray(points), brick_height=0.053)

        np.testing.assert_allclose(pose[:2], center_xy, atol=1e-6)
        self.assertAlmostEqual(float(pose[2]), 0.1, places=6)
        self.assertLess(yaw_error_mod_pi(float(pose[3]), yaw), 1e-6)

    def test_yaw_error_treats_half_turn_as_equivalent(self):
        self.assertAlmostEqual(yaw_error_mod_pi(0.1 + math.pi, 0.1), 0.0)
        self.assertAlmostEqual(yaw_error_mod_pi(-math.pi / 2, math.pi / 2), 0.0)

    def test_pose_fit_uses_top_surface_instead_of_oblique_side_points(self):
        yaw = math.radians(-35.0)
        major = np.array([math.cos(yaw), math.sin(yaw)])
        minor = np.array([-major[1], major[0]])
        center = np.array([-0.1, -0.12])
        points = []
        for along in np.linspace(-0.10, 0.10, 17):
            for across in np.linspace(-0.05, 0.05, 9):
                xy = center + along * major + across * minor
                points.append([xy[0], xy[1], 0.053])
        # A dense visible side has a different apparent covariance and must not
        # steer the horizontal OBB/PCA axis.
        for along in np.linspace(-0.05, 0.05, 45):
            for height in np.linspace(0.0, 0.035, 8):
                xy = center + 0.10 * major + along * minor
                points.append([xy[0], xy[1], height])

        pose = estimate_brick_pose(np.asarray(points), brick_height=0.053)

        np.testing.assert_allclose(pose[:2], center, atol=0.005)
        self.assertLess(yaw_error_mod_pi(pose[3], yaw), math.radians(2.0))


if __name__ == "__main__":
    unittest.main()
