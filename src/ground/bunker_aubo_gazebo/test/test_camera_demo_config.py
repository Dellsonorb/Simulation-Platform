#!/usr/bin/env python
from __future__ import division

import math
import unittest
import xml.etree.ElementTree as ET

import rospkg
import rospy
import rostest
import yaml


class CameraDemoConfigTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.package = rospkg.RosPack().get_path('bunker_aubo_gazebo')

    def test_observation_pose_is_complete_and_safe(self):
        with open(self.package + '/config/camera_observation.yaml') as stream:
            config = yaml.safe_load(stream)
        expected = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.assertEqual(expected, config['joint_names'])
        self.assertEqual(6, len(config['positions']))
        self.assertTrue(all(not math.isnan(value) and not math.isinf(value)
                            for value in config['positions']))
        self.assertGreater(config['duration'], 0.0)
        self.assertNotIn('brick_pose', str(config))

    def test_demo_keeps_running_and_does_not_trigger_pick(self):
        launch_path = self.package + '/launch/camera_demo.launch'
        launch = ET.parse(launch_path).getroot()
        observation = launch.find("node[@name='move_to_camera_observation']")
        self.assertIsNotNone(observation)
        self.assertNotEqual('true', observation.get('required', 'false'))
        include = launch.find('include')
        enabled = include.find("arg[@name='camera_enable_rgbd']")
        self.assertEqual('true', enabled.get('value'))
        with open(launch_path) as stream:
            self.assertNotIn('/brick_pose', stream.read())


if __name__ == '__main__':
    rospy.init_node('test_camera_demo_config')
    rostest.rosrun('bunker_aubo_gazebo', 'test_camera_demo_config',
                   CameraDemoConfigTest)
