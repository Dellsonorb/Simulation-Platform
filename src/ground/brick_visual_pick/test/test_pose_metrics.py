#!/usr/bin/env python
from __future__ import division

import math
import unittest

import tf.transformations as transformations
from geometry_msgs.msg import Pose

from brick_visual_pick.pose_metrics import pose_error


def make_pose(x, y, z, yaw):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    quaternion = transformations.quaternion_from_euler(0.0, 0.0, yaw)
    (pose.orientation.x, pose.orientation.y,
     pose.orientation.z, pose.orientation.w) = quaternion
    return pose


class PoseMetricsTest(unittest.TestCase):
    def test_reports_position_components_and_mod_pi_yaw(self):
        estimate = make_pose(1.003, 2.004, 0.030, -math.pi / 2.0 + 0.01)
        reference = make_pose(1.0, 2.0, 0.026, math.pi / 2.0)
        result = pose_error(estimate, reference)
        self.assertAlmostEqual(result['xy'], 0.005, places=9)
        self.assertAlmostEqual(result['z'], 0.004, places=9)
        self.assertAlmostEqual(result['position'], math.sqrt(0.000041), places=9)
        self.assertAlmostEqual(result['yaw'], 0.01, places=9)


if __name__ == '__main__':
    unittest.main()
