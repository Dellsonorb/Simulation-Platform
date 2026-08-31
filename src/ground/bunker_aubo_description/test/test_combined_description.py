#!/usr/bin/env python
from __future__ import division

import math
import re
import subprocess
import unittest
import xml.etree.ElementTree as ET

import rospy
import rostest
import rospkg
from urdf_parser_py.urdf import URDF


class CombinedDescriptionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = URDF.from_parameter_server('/robot_description')

    def test_single_root_and_required_links(self):
        self.assertEqual('world', self.robot.get_root())
        required = {
            'base_link', 'aubo_mount_link', 'aubo_i5_base_link', 'ee_link',
            'ag95_base_link', 'left_finger_pad', 'right_finger_pad',
            'd435_mount_link', 'camera_link', 'camera_color_optical_frame',
            'camera_depth_optical_frame'}
        self.assertTrue(required.issubset(set(self.robot.link_map)))
        children = [joint.child for joint in self.robot.joints]
        self.assertEqual(len(children), len(set(children)))
        valid = re.compile(r'^[A-Za-z][A-Za-z0-9_/]*$')
        self.assertTrue(all(valid.match(link.name) for link in self.robot.links))
        self.assertTrue(all(valid.match(joint.name) for joint in self.robot.joints))
        wheel_joints = [joint for joint in self.robot.joints
                        if joint.name.startswith('wheel')]
        self.assertEqual(16, len(wheel_joints))
        self.assertTrue(all(joint.type == 'fixed' for joint in wheel_joints))

    def test_mount_chain_and_default_parameters(self):
        parents = {joint.child: joint.parent for joint in self.robot.joints}
        self.assertEqual('base_link', parents['aubo_mount_link'])
        self.assertEqual('aubo_mount_link', parents['aubo_i5_base_link'])
        self.assertEqual('ee_link', parents['ag95_base_link'])
        self.assertEqual('ee_link', parents['d435_mount_link'])
        mount = self.robot.joint_map['aubo_mount_joint'].origin
        self.assertAlmostEqual(0.15, mount.xyz[0])
        # The 80 mm mount cylinder must sit on the BUNKER front deck.  The
        # deck is approximately z=0.001 in base_link, so its center is 42 mm.
        self.assertAlmostEqual(0.042, mount.xyz[2])
        self.assertLess(abs((mount.xyz[2] - 0.04) - 0.001), 0.002)
        self.assertAlmostEqual(0.36,
                               self.robot.joint_map['world_to_odom'].origin.xyz[2])

    def test_optical_frames_use_ros_convention(self):
        for name in ('camera_color_optical_joint', 'camera_depth_optical_joint'):
            rpy = self.robot.joint_map[name].origin.rpy
            self.assertAlmostEqual(-math.pi / 2.0, rpy[0], places=5)
            self.assertAlmostEqual(0.0, rpy[1], places=5)
            self.assertAlmostEqual(-math.pi / 2.0, rpy[2], places=5)

    def test_d435_gazebo_sensors_publish_realsense_style_topics(self):
        package = rospkg.RosPack().get_path('bunker_aubo_description')
        output = subprocess.check_output([
            package + '/scripts/sanitize_urdf_names.py',
            package + '/urdf/bunker_aubo.urdf.xacro',
            'camera_enable_rgbd:=true'])
        root = ET.fromstring(output)
        gazebo = root.find("gazebo[@reference='camera_link']")
        self.assertIsNotNone(gazebo)

        sensors = {sensor.get('name'): sensor for sensor in gazebo.findall('sensor')}
        self.assertEqual('camera', sensors['d435_color'].get('type'))
        self.assertEqual('depth', sensors['d435_depth'].get('type'))

        color_plugin = sensors['d435_color'].find("plugin[@name='d435_color_controller']")
        depth_plugin = sensors['d435_depth'].find("plugin[@name='d435_depth_controller']")
        self.assertIsNotNone(color_plugin)
        self.assertIsNotNone(depth_plugin)
        self.assertEqual('libgazebo_ros_camera.so', color_plugin.get('filename'))
        self.assertEqual('libgazebo_ros_openni_kinect.so', depth_plugin.get('filename'))
        self.assertEqual('true', color_plugin.findtext('alwaysOn'))
        self.assertEqual('true', depth_plugin.findtext('alwaysOn'))
        self.assertEqual('/camera/color/image_raw', color_plugin.findtext('imageTopicName'))
        self.assertEqual('/camera/color/camera_info', color_plugin.findtext('cameraInfoTopicName'))
        self.assertEqual('camera_color_optical_frame', color_plugin.findtext('frameName'))
        self.assertEqual('/camera/depth/image_rect_raw', depth_plugin.findtext('depthImageTopicName'))
        self.assertEqual('/camera/depth/camera_info', depth_plugin.findtext('depthImageCameraInfoTopicName'))
        self.assertEqual('camera_depth_optical_frame', depth_plugin.findtext('frameName'))
        self.assertEqual('0.050', depth_plugin.findtext('hackBaseline'))

        default_root = ET.fromstring(rospy.get_param('/robot_description'))
        self.assertIsNone(default_root.find("gazebo[@reference='camera_link']"))

    def test_arm_and_gripper_have_transmissions(self):
        joints = {joint.name for transmission in self.robot.transmissions
                  for joint in transmission.joints}
        self.assertIn('shoulder_pan_joint', joints)
        self.assertIn('wrist_3_joint', joints)
        self.assertIn('left_outer_knuckle_joint', joints)

    def test_sanitizer_forwards_xacro_camera_overrides(self):
        package = rospkg.RosPack().get_path('bunker_aubo_description')
        output = subprocess.check_output([
            package + '/scripts/sanitize_urdf_names.py',
            package + '/urdf/bunker_aubo.urdf.xacro',
            'camera_enable_rgbd:=true',
            'camera_topic_prefix:=/test_camera',
            'camera_image_width:=320'])
        root = ET.fromstring(output)
        gazebo = root.find("gazebo[@reference='camera_link']")
        plugin = gazebo.find("sensor[@name='d435_color']/plugin")
        self.assertEqual('/test_camera/color/image_raw',
                         plugin.findtext('imageTopicName'))
        self.assertEqual('320', gazebo.findtext(
            "sensor[@name='d435_color']/camera/image/width"))

    def test_reality_check_can_lock_infeasible_ag95_open(self):
        package = rospkg.RosPack().get_path('bunker_aubo_description')
        output = subprocess.check_output([
            package + '/scripts/sanitize_urdf_names.py',
            package + '/urdf/bunker_aubo.urdf.xacro',
            'lock_ag95_open:=true'])
        root = ET.fromstring(output)
        loop_joints = [
            joint.get('name')
            for gazebo in root.findall('gazebo')
            for joint in gazebo.findall('joint')]
        self.assertNotIn('left_inner_knuckle_to_finger_joint', loop_joints)
        self.assertNotIn('right_inner_knuckle_to_finger_joint', loop_joints)
        mimic_plugins = []
        for gazebo in root.findall('gazebo'):
            for plugin in gazebo.findall('plugin'):
                if plugin.get('filename') == \
                        'libroboticsgroup_upatras_gazebo_mimic_joint_plugin.so':
                    mimic_plugins.append(plugin)
        self.assertEqual([], mimic_plugins)
        passive = {
            'right_outer_knuckle_joint', 'left_finger_joint',
            'right_finger_joint', 'left_inner_knuckle_joint',
            'right_inner_knuckle_joint'}
        joints = {joint.get('name'): joint for joint in root.findall('joint')}
        self.assertEqual('revolute', joints['left_outer_knuckle_joint'].get('type'))
        self.assertEqual(passive,
                         {name for name in passive
                          if joints[name].get('type') == 'fixed'})

    def test_reality_check_can_use_bounded_effort_ag95_without_loop_shortcut(self):
        package = rospkg.RosPack().get_path('bunker_aubo_description')
        output = subprocess.check_output([
            package + '/scripts/sanitize_urdf_names.py',
            package + '/urdf/bunker_aubo.urdf.xacro',
            'ag95_effort_coupled:=true'])
        root = ET.fromstring(output)
        loop_names = {
            joint.get('name')
            for gazebo in root.findall('gazebo')
            for joint in gazebo.findall('joint')}
        self.assertNotIn('left_inner_knuckle_to_finger_joint', loop_names)
        self.assertNotIn('right_inner_knuckle_to_finger_joint', loop_names)
        plugins = [
            plugin for gazebo in root.findall('gazebo')
            for plugin in gazebo.findall('plugin')]
        self.assertFalse(any(
            plugin.get('filename') ==
            'libroboticsgroup_upatras_gazebo_mimic_joint_plugin.so'
            for plugin in plugins))
        coupling = [plugin for plugin in plugins
                    if plugin.get('filename') ==
                    'libag95_effort_coupling_plugin.so']
        self.assertEqual(1, len(coupling))
        self.assertEqual('left_outer_knuckle_joint',
                         coupling[0].findtext('masterJoint'))
        self.assertEqual('50.0', coupling[0].findtext('maximumEffort'))
        joints = {joint.get('name'): joint for joint in root.findall('joint')}
        self.assertEqual('0.93', joints['left_outer_knuckle_joint'].find(
            'limit').get('upper'))
        self.assertEqual('revolute',
                         joints['right_outer_knuckle_joint'].get('type'))
        self.assertEqual('left_outer_knuckle_joint',
                         joints['right_outer_knuckle_joint'].find(
                             'mimic').get('joint'))
        fixed_helpers = {
            'left_finger_joint', 'right_finger_joint',
            'left_inner_knuckle_joint', 'right_inner_knuckle_joint'}
        self.assertEqual(fixed_helpers, {
            name for name in fixed_helpers
            if joints[name].get('type') == 'fixed'})
        preserved = {
            gazebo.get('reference') for gazebo in root.findall('gazebo')
            if gazebo.findtext('preserveFixedJoint') == 'true'}
        self.assertTrue(fixed_helpers.issubset(preserved))
        self.assertTrue({
            'left_inner_finger_pad_joint',
            'right_inner_finger_pad_joint'}.issubset(preserved))

    def test_ag95_compatibility_modes_are_mutually_exclusive(self):
        package = rospkg.RosPack().get_path('bunker_aubo_description')
        with self.assertRaises(subprocess.CalledProcessError):
            subprocess.check_output([
                package + '/scripts/sanitize_urdf_names.py',
                package + '/urdf/bunker_aubo.urdf.xacro',
                'lock_ag95_open:=true', 'ag95_effort_coupled:=true'],
                stderr=subprocess.STDOUT)

    def test_attachment_opening_is_derived_from_brick_orientation(self):
        package = rospkg.RosPack().get_path('bunker_aubo_description')

        def required(mode):
            output = subprocess.check_output([
                package + '/scripts/sanitize_urdf_names.py',
                package + '/urdf/bunker_aubo.urdf.xacro',
                'brick_orientation_mode:=' + mode,
                'brick_opening_clearance:=0.002'])
            root = ET.fromstring(output)
            plugins = [
                plugin for gazebo in root.findall('gazebo')
                for plugin in gazebo.findall('plugin')
                if plugin.get('filename') ==
                'libguarded_brick_attachment.so']
            self.assertEqual(1, len(plugins))
            return float(plugins[0].findtext('requiredOpening'))

        self.assertAlmostEqual(0.117, required('flat'))
        self.assertAlmostEqual(0.055, required('side_up'))


if __name__ == '__main__':
    rospy.init_node('test_combined_description')
    rostest.rosrun('bunker_aubo_description', 'test_combined_description',
                   CombinedDescriptionTest)
