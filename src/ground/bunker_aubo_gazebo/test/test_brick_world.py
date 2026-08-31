#!/usr/bin/env python
from __future__ import division

import math
import os
import subprocess
import unittest

import rospy
import rostest
from gazebo_msgs.srv import GetModelState
from urdf_parser_py.urdf import URDF


class BrickWorldTest(unittest.TestCase):
    @staticmethod
    def expand_brick(mode):
        xacro_path = os.path.abspath(os.path.join(
            os.path.dirname(__file__), '..', 'urdf', 'brick.urdf.xacro'))
        return subprocess.check_output([
            'xacro', '--inorder', xacro_path,
            'length:=0.240', 'width:=0.115', 'height:=0.053',
            'mass:=2.5', 'orientation_mode:=' + mode,
        ])

    def test_orientation_modes_preserve_nominal_size_but_orient_collision(self):
        flat = URDF.from_xml_string(self.expand_brick('flat'))
        side = URDF.from_xml_string(self.expand_brick('side_up'))
        self.assertEqual([0.240, 0.115, 0.053],
                         flat.link_map['brick_link'].collision.geometry.size)
        self.assertEqual([0.240, 0.053, 0.115],
                         side.link_map['brick_link'].collision.geometry.size)
        self.assertAlmostEqual(
            2.5 * (0.053**2 + 0.115**2) / 12.0,
            side.link_map['brick_link'].inertial.inertia.ixx, places=8)

    def test_brick_physics_and_spawn_pose(self):
        robot = URDF.from_parameter_server('/brick_description')
        link = robot.link_map['brick_link']
        self.assertEqual([0.240, 0.115, 0.053], link.collision.geometry.size)
        self.assertAlmostEqual(2.5, link.inertial.mass)
        self.assertAlmostEqual(2.5 * (0.115**2 + 0.053**2) / 12.0,
                               link.inertial.inertia.ixx, places=8)
        xml = rospy.get_param('/brick_description')
        self.assertIn('<gravity>true</gravity>', xml)
        self.assertIn('<mu1>1.2</mu1>', xml)
        self.assertIn('<material>Gazebo/Red</material>', xml)

        rospy.wait_for_service('/gazebo/get_model_state', timeout=30.0)
        get_model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        deadline = rospy.Time.now() + rospy.Duration(20.0)
        state = get_model('brick', 'world')
        while not state.success and rospy.Time.now() < deadline:
            rospy.sleep(0.1)
            state = get_model('brick', 'world')
        self.assertTrue(state.success, state.status_message)
        rospy.sleep(1.0)
        state = get_model('brick', 'world')
        self.assertTrue(all(not math.isnan(value) and not math.isinf(value)
                            for value in (state.pose.position.x,
                                          state.pose.position.y,
                                          state.pose.position.z)))
        self.assertAlmostEqual(0.75, state.pose.position.x, delta=0.03)
        self.assertAlmostEqual(0.10, state.pose.position.y, delta=0.03)
        self.assertGreater(state.pose.position.z, 0.015)


if __name__ == '__main__':
    rospy.init_node('test_brick_world')
    rostest.rosrun('bunker_aubo_gazebo', 'test_brick_world', BrickWorldTest)
