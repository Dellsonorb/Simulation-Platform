#!/usr/bin/env python
from __future__ import division

import math
import unittest

from brick_rgbd_perception.brick_geometry import (
    min_cloud_points_for_geometry, oriented_brick_geometry)


class BrickGeometryTest(unittest.TestCase):
    def test_flat_and_side_up_are_derived_from_nominal_dimensions(self):
        flat = oriented_brick_geometry(0.240, 0.115, 0.053, 'flat')
        self.assertEqual((0.240, 0.115, 0.053, 0.115, 0.0265),
                         (flat.top_length, flat.top_width,
                          flat.vertical_height, flat.grasp_span,
                          flat.resting_center_z))

        side = oriented_brick_geometry(0.240, 0.115, 0.053, 'side_up')
        self.assertEqual((0.240, 0.053, 0.115, 0.053, 0.0575),
                         (side.top_length, side.top_width,
                          side.vertical_height, side.grasp_span,
                          side.resting_center_z))

    def test_invalid_mode_or_dimensions_fail_closed(self):
        assert_raises_regex = (getattr(self, 'assertRaisesRegex', None) or
                               getattr(self, 'assertRaisesRegexp'))
        with assert_raises_regex(ValueError,
                                 'INVALID_BRICK_ORIENTATION_MODE'):
            oriented_brick_geometry(0.240, 0.115, 0.053, 'edge')
        for dimensions in ((float('nan'), 0.115, 0.053),
                           (0.240, 0.0, 0.053),
                           (0.100, 0.115, 0.053)):
            with assert_raises_regex(ValueError,
                                     'INVALID_BRICK_DIMENSIONS'):
                oriented_brick_geometry(*(dimensions + ('side_up',)))

    def test_minimum_cloud_points_scale_with_visible_top_area(self):
        flat = oriented_brick_geometry(0.240, 0.115, 0.053, 'flat')
        side = oriented_brick_geometry(0.240, 0.115, 0.053, 'side_up')
        self.assertEqual(60000, min_cloud_points_for_geometry(
            60000, 0.240 * 0.115, flat))
        self.assertEqual(27653, min_cloud_points_for_geometry(
            60000, 0.240 * 0.115, side))
        assert_raises_regex = (getattr(self, 'assertRaisesRegex', None) or
                               getattr(self, 'assertRaisesRegexp'))
        with assert_raises_regex(ValueError, 'INVALID_CLOUD_REFERENCE'):
            min_cloud_points_for_geometry(0, 0.240 * 0.115, side)


if __name__ == '__main__':
    unittest.main()
