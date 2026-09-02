#!/usr/bin/env python
from __future__ import division

import os
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT = os.path.dirname(PACKAGE)


class ConfigurationTest(unittest.TestCase):
    def read(self, relative):
        with open(os.path.join(PACKAGE, relative)) as stream:
            return stream.read()

    def test_navigation_launch_is_isolated_from_visual_pick(self):
        text = self.read('launch/navigation_demo.launch')
        root = ET.fromstring(text)
        self.assertIn('move_base', text)
        self.assertIn('mobile_base', text)
        self.assertNotIn('brick_visual_pick', text)
        self.assertNotIn('brick_pick_demo', text)
        self.assertEqual(len(root.findall(".//node[@type='move_base']")), 1)

    def test_navigation_launch_freezes_manipulator_for_navigation(self):
        text = self.read('launch/navigation_demo.launch')
        root = ET.fromstring(text)
        controller_args = root.findall(
            ".//include/arg[@name='spawn_controllers']")
        self.assertEqual(len(controller_args), 1)
        self.assertEqual(controller_args[0].attrib.get('value'), 'false')
        publishers = root.findall(".//node[@type='joint_state_publisher']")
        self.assertEqual(len(publishers), 1)

    def test_mobile_base_is_opt_in_and_uses_upstream_control_contract(self):
        path = os.path.join(PROJECT, 'bunker_aubo_description', 'urdf',
                            'bunker_aubo.urdf.xacro')
        with open(path) as stream:
            text = stream.read()
        self.assertIn('<xacro:arg name="mobile_base" default="false"/>', text)
        self.assertIn('libbunker_planar_drive_plugin.so', text)
        self.assertIn('<commandTopic>/cmd_vel</commandTopic>', text)
        self.assertIn('<odometryTopic>/odom</odometryTopic>', text)

    def test_planar_drive_keeps_articulated_upper_body_kinematic(self):
        text = self.read('src/bunker_planar_drive_plugin.cpp')
        self.assertIn('link->SetKinematic(true);', text)
        self.assertIn('model_->SetWorldPose(pose_);', text)
        self.assertNotIn('link_offsets_', text)

    def test_move_base_uses_shared_frames_and_non_holonomic_local_planner(self):
        with open(os.path.join(PACKAGE, 'config', 'costmap_common.yaml')) as stream:
            common = yaml.safe_load(stream)
        with open(os.path.join(PACKAGE, 'config', 'move_base.yaml')) as stream:
            move_base = yaml.safe_load(stream)
        with open(os.path.join(PACKAGE, 'config', 'local_planner.yaml')) as stream:
            local = yaml.safe_load(stream)['DWAPlannerROS']
        with open(os.path.join(PACKAGE, 'config',
                               'global_costmap.yaml')) as stream:
            global_map = yaml.safe_load(stream)['global_costmap']
        with open(os.path.join(PACKAGE, 'config',
                               'local_costmap.yaml')) as stream:
            local_map = yaml.safe_load(stream)['local_costmap']
        self.assertNotIn('global_frame', common)
        self.assertEqual(common['robot_base_frame'], 'ground/base_link')
        self.assertEqual(global_map['global_frame'], 'map')
        self.assertEqual(local_map['global_frame'], 'ground/odom')
        self.assertEqual(move_base['base_global_planner'], 'navfn/NavfnROS')
        self.assertEqual(move_base['base_local_planner'],
                         'dwa_local_planner/DWAPlannerROS')
        self.assertFalse(local['holonomic_robot'])
        self.assertEqual(local['odom_topic'], '/ground/odom')

    def test_rviz_exposes_2d_navigation_goal_tool(self):
        with open(os.path.join(PACKAGE, 'rviz', 'navigation.rviz')) as stream:
            config = yaml.safe_load(stream)
        tools = config['Visualization Manager']['Tools']
        goal_tools = [tool for tool in tools
                      if tool.get('Class') == 'rviz/SetGoal']
        self.assertEqual(len(goal_tools), 1)
        self.assertEqual(goal_tools[0].get('Topic'),
                         '/move_base_simple/goal')

    def test_heading_gate_owns_cmd_vel_and_preserves_reverse(self):
        root = ET.fromstring(self.read('launch/navigation_demo.launch'))
        move_base = root.find(".//node[@type='move_base']")
        remaps = move_base.findall("remap[@from='cmd_vel']")
        self.assertEqual([item.attrib['to'] for item in remaps],
                         ['/navigation/raw_cmd_vel'])
        gate = root.find(".//node[@type='heading_gate_node.py']")
        self.assertIsNotNone(gate)
        self.assertEqual(gate.attrib.get('required'), 'true')
        with open(os.path.join(PACKAGE, 'config',
                               'local_planner.yaml')) as stream:
            local = yaml.safe_load(stream)['DWAPlannerROS']
        self.assertLess(local['min_vel_x'], 0.0)

    def test_heading_gate_uses_goal_ids_for_lifecycle_reset(self):
        text = self.read('scripts/heading_gate_node.py')
        self.assertIn("Subscriber('/move_base/goal'", text)
        self.assertIn("Subscriber('/move_base/result'", text)
        self.assertNotIn("Subscriber('/move_base/current_goal'", text)

    def test_navigation_laser_is_opt_in_and_enabled_only_for_navigation(self):
        description = os.path.join(PROJECT, 'bunker_aubo_description', 'urdf',
                                   'bunker_aubo.urdf.xacro')
        with open(description) as stream:
            text = stream.read()
        self.assertIn('<xacro:arg name="navigation_laser" default="false"/>',
                      text)
        self.assertIn('libgazebo_ros_laser.so', text)
        launch = self.read('launch/navigation_demo.launch')
        self.assertIn('<arg name="navigation_laser" value="true"/>', launch)

    def test_costmaps_use_scan_obstacle_layer_and_publish_full_local_map(self):
        with open(os.path.join(PACKAGE, 'config',
                               'costmap_common.yaml')) as stream:
            common = yaml.safe_load(stream)
        self.assertEqual(common['obstacle_layer']['observation_sources'],
                         'scan')
        scan = common['obstacle_layer']['scan']
        self.assertEqual(scan['topic'], '/ground/scan')
        self.assertEqual(scan['sensor_frame'], 'ground/lidar_2d_link')
        self.assertEqual(scan['data_type'], 'LaserScan')
        self.assertTrue(scan['marking'])
        self.assertTrue(scan['clearing'])
        for name in ('local_costmap.yaml', 'global_costmap.yaml'):
            with open(os.path.join(PACKAGE, 'config', name)) as stream:
                config = yaml.safe_load(stream)
            root = config[name[:-5]]
            types = [item['type'] for item in root['plugins']]
            self.assertIn('costmap_2d::ObstacleLayer', types)
        with open(os.path.join(PACKAGE, 'config',
                               'local_costmap.yaml')) as stream:
            local = yaml.safe_load(stream)['local_costmap']
        self.assertTrue(local['always_send_full_costmap'])
        with open(os.path.join(PACKAGE, 'config',
                               'global_costmap.yaml')) as stream:
            global_map = yaml.safe_load(stream)['global_costmap']
        self.assertTrue(global_map['always_send_full_costmap'])

    def test_gate_consumes_local_costmap_and_shared_footprint(self):
        launch = self.read('launch/navigation_demo.launch')
        self.assertIn('costmap_topic', launch)
        self.assertIn('/move_base/local_costmap/costmap', launch)
        self.assertIn('costmap_common.yaml', launch)
        node = self.read('scripts/heading_gate_node.py')
        self.assertIn('OccupancyGrid', node)
        self.assertIn('check_rotation_sweep', node)
        self.assertIn('self.gate.block()', node)

    def test_heading_gate_runtime_dependencies_are_declared(self):
        package = ET.parse(os.path.join(PACKAGE, 'package.xml')).getroot()
        exec_dependencies = [item.text
                             for item in package.findall('exec_depend')]
        self.assertIn('std_msgs', exec_dependencies)
        cmake = self.read('CMakeLists.txt')
        self.assertIn('scripts/heading_gate_node.py', cmake)

    def test_approach_demo_reuses_navigation_without_manipulation(self):
        text = self.read('launch/approach_pose_demo.launch')
        root = ET.fromstring(text)
        includes = root.findall('.//include')
        self.assertTrue(any('navigation_demo.launch' in
                            item.attrib.get('file', '') for item in includes))
        node = root.find(".//node[@type='approach_pose_node.py']")
        self.assertIsNotNone(node)
        self.assertEqual(node.attrib.get('required'), 'true')
        self.assertNotIn('brick_visual_pick', text)
        self.assertNotIn('brick_pick_demo', text)
        self.assertNotIn('/brick_pose', text)

    def test_approach_node_uses_global_costmap_and_existing_move_base(self):
        text = self.read('scripts/approach_pose_node.py')
        self.assertIn('/move_base/global_costmap/costmap', text)
        self.assertIn('/move_base/make_plan', text)
        self.assertIn("SimpleActionClient('/move_base'", text)
        self.assertIn('candidate_is_clear', text)
        self.assertIn("'NO_SAFE_CANDIDATE'", text)
        self.assertNotIn('/cmd_vel', text)
        self.assertNotIn('/brick_pose', text)

    def test_approach_parameters_match_verified_arm_workspace(self):
        with open(os.path.join(PACKAGE, 'config',
                               'approach_pose.yaml')) as stream:
            config = yaml.safe_load(stream)
        self.assertAlmostEqual(config['work_distance'], 0.82)
        self.assertAlmostEqual(config['arm_offset_x'], 0.15)
        self.assertLessEqual(config['reach_min'], config['work_distance'])
        self.assertGreaterEqual(config['reach_max'], config['work_distance'])
        self.assertEqual(config['input_topic'], '/known_brick_pose')
        self.assertEqual(config['output_topic'], '/bunker/approach_pose')
        self.assertEqual(config['direct_drive_bearing_limit_deg'], 30.0)
        self.assertAlmostEqual(config['brick_length'], 0.240)
        self.assertAlmostEqual(config['brick_width'], 0.115)
        self.assertGreater(config['target_safety_padding'], 0.0)
        self.assertGreater(config['target_sweep_linear_step'], 0.0)
        self.assertGreater(config['target_sweep_angular_step'], 0.0)
        self.assertGreaterEqual(config['costmap_settle_duration'], 0.8)

    def test_approach_runtime_files_are_installed(self):
        cmake = self.read('CMakeLists.txt')
        self.assertIn('scripts/approach_pose_node.py', cmake)
        package = ET.parse(os.path.join(PACKAGE, 'package.xml')).getroot()
        dependencies = [item.text for item in package.findall('exec_depend')]
        self.assertIn('visualization_msgs', dependencies)


if __name__ == '__main__':
    unittest.main()
