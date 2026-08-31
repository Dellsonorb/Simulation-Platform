#!/usr/bin/env python
from __future__ import division

import os
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT = os.path.dirname(PACKAGE)


class ConfigurationTest(unittest.TestCase):
    def test_visual_launch_has_independent_pipeline_and_no_manual_pose(self):
        launch = os.path.join(PACKAGE, 'launch', 'visual_pick_demo.launch')
        root = ET.parse(launch).getroot()
        with open(launch) as stream:
            text = stream.read()
        self.assertIn('brick_rgbd_perception', text)
        self.assertIn('pose_estimator.launch', text)
        self.assertIn('brick_pose_gate.py', text)
        self.assertIn('brick_pick.launch', text)
        self.assertNotIn('manual_brick_pose_publisher', text)
        self.assertEqual(len(root.findall(".//node[@type='brick_pose_gate.py']")), 1)
        mover = root.find(".//node[@name='move_to_camera_observation']")
        self.assertIn('sleep 2', mover.attrib.get('launch-prefix', ''))

    def test_scenario_matrix_contains_twenty_distinct_valid_cases_and_control(self):
        path = os.path.join(PACKAGE, 'config', 'visual_pick_scenarios.yaml')
        with open(path) as stream:
            data = yaml.safe_load(stream)
        valid = data['valid_scenarios']
        self.assertGreaterEqual(len(valid), 20)
        poses = set((item['x'], item['y'], item['yaw']) for item in valid)
        self.assertEqual(len(poses), len(valid))
        self.assertFalse(data['rejection_control']['expect_pose'])

    def test_gate_requires_robot_base_frame(self):
        path = os.path.join(PACKAGE, 'config', 'pose_gate.yaml')
        with open(path) as stream:
            data = yaml.safe_load(stream)
        self.assertEqual(data['required_frame'], 'aubo_i5_base_link')
        self.assertEqual(data['required_samples'], 6)
        self.assertGreater(data['minimum_sample_span'], 0.0)

    def test_v01_pick_launch_default_remains_world_but_can_be_overridden(self):
        path = os.path.join(PROJECT, 'brick_pick_demo', 'launch',
                            'brick_pick.launch')
        root = ET.parse(path).getroot()
        planning_arg = root.find("arg[@name='planning_frame']")
        self.assertIsNotNone(planning_arg)
        self.assertEqual(planning_arg.attrib['default'], 'world')
        override = root.find("param[@name='brick_pick_node/grasp/planning_frame']")
        self.assertEqual(override.attrib['value'], '$(arg planning_frame)')

    def test_matrix_acceptance_cannot_skip_required_trials_or_control(self):
        path = os.path.join(os.path.dirname(PROJECT), '..', 'scripts',
                            'run_visual_pick_matrix.py')
        with open(os.path.abspath(path)) as stream:
            text = stream.read()
        self.assertNotIn("add_argument('--limit'", text)
        self.assertNotIn("add_argument('--skip-control'", text)
        self.assertIn('result_path.unlink()', text)


if __name__ == '__main__':
    unittest.main()
