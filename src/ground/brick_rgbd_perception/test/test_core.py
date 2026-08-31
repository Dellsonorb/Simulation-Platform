#!/usr/bin/env python
from __future__ import division

import unittest

import cv2
import numpy as np

from brick_rgbd_perception.core import backproject_mask, segment_hsv
from brick_rgbd_perception.pose_estimation import (
    estimate_box_4dof, yaw_error_mod_pi)


class CoreTest(unittest.TestCase):
    def test_segment_hsv_keeps_largest_red_component(self):
        rgb = np.full((60, 80, 3), 90, dtype=np.uint8)
        rgb[20:45, 25:65] = (255, 0, 0)
        rgb[3:7, 3:7] = (255, 0, 0)

        mask = segment_hsv(
            rgb,
            hue_ranges=((0, 10), (170, 179)),
            saturation_min=100,
            value_min=40,
            morphology_kernel=3,
            min_component_pixels=100,
        )

        self.assertEqual(np.uint8, mask.dtype)
        self.assertEqual(255, int(mask[30, 40]))
        self.assertEqual(0, int(mask[4, 4]))
        self.assertEqual(1000, cv2.countNonZero(mask))

    def test_backproject_uses_same_pixels_and_filters_bad_depth(self):
        depth = np.array([
            [0.0, 1.0, np.nan, 6.0],
            [np.inf, 2.0, 0.05, 3.0],
        ], dtype=np.float32)
        mask = np.full(depth.shape, 255, dtype=np.uint8)
        camera_k = np.array([
            [2.0, 0.0, 1.0],
            [0.0, 4.0, 0.5],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)

        points, pixels = backproject_mask(
            depth, mask, camera_k, min_depth=0.1, max_depth=5.0,
            pixel_stride=1,
        )

        expected_pixels = np.array([[1, 0], [1, 1], [3, 1]])
        expected_points = np.array([
            [0.0, -0.125, 1.0],
            [0.0, 0.25, 2.0],
            [3.0, 0.375, 3.0],
        ], dtype=np.float32)
        np.testing.assert_array_equal(expected_pixels, pixels)
        np.testing.assert_allclose(expected_points, points, atol=1e-6)

    def test_backproject_rejects_shape_mismatch(self):
        with self.assertRaises(ValueError):
            backproject_mask(
                np.ones((2, 3), dtype=np.float32),
                np.ones((3, 2), dtype=np.uint8),
                np.eye(3), 0.1, 5.0, 1,
            )

    def test_estimate_box_uses_top_surface_and_known_height(self):
        center = np.array([0.62, -0.18, -0.3485])
        yaw = 0.43
        length, width, height = 0.240, 0.115, 0.053
        long_samples = np.linspace(-length / 2.0, length / 2.0, 49)
        short_samples = np.linspace(-width / 2.0, width / 2.0, 25)
        local_xy = np.array([(x, y) for x in long_samples
                             for y in short_samples])
        rotation = np.array([[np.cos(yaw), -np.sin(yaw)],
                             [np.sin(yaw), np.cos(yaw)]])
        top_xy = np.dot(local_xy, rotation.T) + center[:2]
        top = np.column_stack((
            top_xy, np.full(top_xy.shape[0], center[2] + height / 2.0)))

        # Add a heavily sampled visible side: a raw 3-D mean is deliberately
        # biased downward and toward one edge.
        side_z = np.linspace(center[2] - height / 2.0,
                             center[2] + height / 2.0 - 0.012, 18)
        side_local = np.array([(x, width / 2.0, z)
                               for x in long_samples for z in side_z])
        side_xy = np.dot(side_local[:, :2], rotation.T) + center[:2]
        side = np.column_stack((side_xy, side_local[:, 2]))
        points = np.vstack((top, side))

        estimate = estimate_box_4dof(
            points, length, width, height, top_surface_tolerance=0.006,
            min_top_points=100)

        np.testing.assert_allclose(estimate[:3], center, atol=0.002)
        self.assertLess(yaw_error_mod_pi(estimate[3], yaw), 0.01)
        self.assertGreater(np.linalg.norm(points.mean(axis=0) - center), 0.01)

    def test_estimate_side_up_box_uses_exposed_top_and_vertical_height(self):
        center = np.array([0.64, 0.07, -0.3425])
        yaw = -0.61
        top_length, top_width, vertical_height = 0.240, 0.053, 0.115
        xs = np.linspace(-top_length / 2.0, top_length / 2.0, 49)
        ys = np.linspace(-top_width / 2.0, top_width / 2.0, 17)
        local = np.array([(x, y) for x in xs for y in ys])
        rotation = np.array([[np.cos(yaw), -np.sin(yaw)],
                             [np.sin(yaw), np.cos(yaw)]])
        xy = np.dot(local, rotation.T) + center[:2]
        points = np.column_stack((
            xy, np.full(xy.shape[0], center[2] + vertical_height / 2.0)))

        estimate = estimate_box_4dof(
            points, top_length, top_width, vertical_height,
            top_surface_tolerance=0.006, min_top_points=100)

        np.testing.assert_allclose(estimate[:3], center, atol=0.002)
        self.assertLess(yaw_error_mod_pi(estimate[3], yaw), 0.01)

    def test_yaw_error_treats_pi_flipped_axis_as_equal(self):
        self.assertAlmostEqual(0.0, yaw_error_mod_pi(0.2, 0.2 + np.pi),
                               places=12)
        self.assertAlmostEqual(0.03, yaw_error_mod_pi(-0.01, 0.02),
                               places=12)

    def test_estimate_box_restores_dimension_when_one_end_is_image_clipped(self):
        center = np.array([0.60, 0.02, -0.4565])
        length, width, height = 0.240, 0.115, 0.053
        # The negative long-axis end is outside the image; 30 mm is missing.
        xs = np.linspace(-0.090, length / 2.0, 43)
        ys = np.linspace(-width / 2.0, width / 2.0, 25)
        local = np.array([(x, y) for x in xs for y in ys])
        points = np.column_stack((
            local + center[:2],
            np.full(local.shape[0], center[2] + height / 2.0)))
        pixels = np.column_stack((
            320.0 + local[:, 1] * 1500.0,
            479.0 - (local[:, 0] + 0.090) / 0.210 * 340.0))

        estimate = estimate_box_4dof(
            points, length, width, height, top_surface_tolerance=0.006,
            min_top_points=100, image_points=pixels,
            image_size=(640, 480), image_border_margin=2.0)

        np.testing.assert_allclose(estimate[:3], center, atol=0.002)
        self.assertLess(yaw_error_mod_pi(estimate[3], 0.0), 0.01)

    def test_estimate_box_rejects_degenerate_or_narrow_top_surface(self):
        collapsed = np.tile(np.array([[0.6, 0.0, -0.43]]), (200, 1))
        with self.assertRaises(ValueError):
            estimate_box_4dof(collapsed, 0.240, 0.115, 0.053)

        xs = np.linspace(-0.12, 0.12, 200)
        narrow = np.column_stack((
            0.6 + xs, np.zeros(xs.shape),
            np.full(xs.shape, -0.43)))
        with self.assertRaises(ValueError):
            estimate_box_4dof(narrow, 0.240, 0.115, 0.053)

    def test_estimate_box_rejects_ambiguous_partial_extent(self):
        xs = np.linspace(-0.090, 0.120, 43)
        ys = np.linspace(-0.0575, 0.0575, 25)
        local = np.array([(x, y) for x in xs for y in ys])
        points = np.column_stack((
            0.6 + local[:, 0], local[:, 1],
            np.full(local.shape[0], -0.43)))
        # The long edge is incomplete, but no endpoint touches an image
        # boundary.  Which physical end is missing is unobservable.
        pixels = np.column_stack((
            320.0 + local[:, 1] * 1200.0,
            300.0 - local[:, 0] * 600.0))
        with self.assertRaises(ValueError):
            estimate_box_4dof(
                points, 0.240, 0.115, 0.053,
                image_points=pixels, image_size=(640, 480))


if __name__ == '__main__':
    unittest.main()
