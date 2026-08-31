#!/usr/bin/env python
from __future__ import division

import math
import unittest

from bunker_navigation.costmap_sweep import (CostmapGrid,
                                             check_rotation_sweep,
                                             pad_footprint)


class CostmapSweepTest(unittest.TestCase):
    FOOTPRINT = ((-0.52, -0.39), (-0.52, 0.39),
                 (0.52, 0.39), (0.52, -0.39))

    @staticmethod
    def grid(obstacles=()):
        resolution = 0.02
        width = height = 200
        origin = -2.0
        data = [0] * (width * height)
        for x, y in obstacles:
            mx = int(math.floor((x - origin) / resolution))
            my = int(math.floor((y - origin) / resolution))
            data[my * width + mx] = 100
        return CostmapGrid(width, height, resolution, origin, origin, data)

    def test_near_side_obstacle_blocks_complete_rear_rotation_sweep(self):
        result = check_rotation_sweep(
            self.grid(((0.0, 0.58),)), self.FOOTPRINT,
            robot_pose=(0.0, 0.0, 0.0), goal_xy=(-1.0, 0.0),
            release_bearing=math.radians(30.0), angular_step=0.025)
        self.assertFalse(result.clear)
        self.assertEqual(result.reason, 'COLLISION')
        self.assertIsNotNone(result.collision_yaw)

    def test_costmap_padding_matches_configured_robot_footprint(self):
        padded = pad_footprint(
            ((-0.50, -0.37), (-0.50, 0.37),
             (0.50, 0.37), (0.50, -0.37)), 0.02)
        self.assertEqual(padded, list(self.FOOTPRINT))

    def test_obstacle_outside_swept_footprint_does_not_false_block(self):
        result = check_rotation_sweep(
            self.grid(((0.0, 1.20),)), self.FOOTPRINT,
            robot_pose=(0.0, 0.0, 0.0), goal_xy=(-1.0, 0.0),
            release_bearing=math.radians(30.0), angular_step=0.025)
        self.assertTrue(result.clear)
        self.assertEqual(result.reason, 'CLEAR')
        self.assertGreater(result.checked_poses, 100)

    def test_inflation_cost_is_not_treated_as_a_second_robot_footprint(self):
        grid = self.grid(((0.0, 0.58),))
        grid.data[grid.data.index(100)] = 99
        result = check_rotation_sweep(
            grid, self.FOOTPRINT, (0.0, 0.0, 0.0), (-1.0, 0.0),
            math.radians(30.0), 0.025)
        self.assertTrue(result.clear)

    def test_unknown_cell_and_outside_map_are_unavailable(self):
        unknown = self.grid()
        unknown.data[100 * unknown.width + 100] = -1
        result = check_rotation_sweep(
            unknown, self.FOOTPRINT, (0.0, 0.0, 0.0), (-1.0, 0.0),
            math.radians(30.0), 0.025)
        self.assertFalse(result.clear)
        self.assertEqual(result.reason, 'UNKNOWN')

        small = CostmapGrid(10, 10, 0.05, -0.25, -0.25, [0] * 100)
        result = check_rotation_sweep(
            small, self.FOOTPRINT, (0.0, 0.0, 0.0), (-1.0, 0.0),
            math.radians(30.0), 0.025)
        self.assertFalse(result.clear)
        self.assertEqual(result.reason, 'OUTSIDE_COSTMAP')


if __name__ == '__main__':
    unittest.main()
