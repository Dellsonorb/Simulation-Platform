#!/usr/bin/env python
from __future__ import division

import math
import unittest

import rospy
import rostest
from gazebo_msgs.srv import GetModelState
from urdf_parser_py.urdf import URDF


class BunkerSpawnTest(unittest.TestCase):
    def setUp(self):
        rospy.wait_for_service('/gazebo/get_model_state', timeout=30.0)
        self.get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    def wait_for_model(self, timeout=20.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        state = self.get_state('bunker', 'world')
        while not state.success and rospy.Time.now() < deadline:
            rospy.sleep(0.1)
            state = self.get_state('bunker', 'world')
        return state

    def test_description_has_base_link(self):
        robot = URDF.from_parameter_server('/robot_description')
        self.assertIn('base_link', [link.name for link in robot.links])

    def test_model_is_finite_and_stationary(self):
        first = self.wait_for_model()
        self.assertTrue(first.success, first.status_message)
        rospy.sleep(3.0)
        second = self.get_state('bunker', 'world')
        self.assertTrue(second.success, second.status_message)
        values = [second.pose.position.x, second.pose.position.y,
                  second.pose.position.z, second.pose.orientation.x,
                  second.pose.orientation.y, second.pose.orientation.z,
                  second.pose.orientation.w]
        self.assertTrue(all(not math.isnan(value) and not math.isinf(value)
                            for value in values))
        displacement = math.sqrt(
            (second.pose.position.x - first.pose.position.x) ** 2 +
            (second.pose.position.y - first.pose.position.y) ** 2 +
            (second.pose.position.z - first.pose.position.z) ** 2)
        self.assertLess(displacement, 1.0e-3)


if __name__ == '__main__':
    rospy.init_node('test_bunker_spawn')
    rostest.rosrun('bunker_aubo_gazebo', 'test_bunker_spawn', BunkerSpawnTest)
