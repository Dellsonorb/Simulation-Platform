#!/usr/bin/env python
from __future__ import division

import os
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT = os.path.dirname(PACKAGE)
WORKSPACE = os.path.dirname(os.path.dirname(PROJECT))


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

    def test_move_base_uses_odom_and_non_holonomic_local_planner(self):
        with open(os.path.join(PACKAGE, 'config', 'costmap_common.yaml')) as stream:
            common = yaml.safe_load(stream)
        with open(os.path.join(PACKAGE, 'config', 'move_base.yaml')) as stream:
            move_base = yaml.safe_load(stream)
        with open(os.path.join(PACKAGE, 'config', 'local_planner.yaml')) as stream:
            local = yaml.safe_load(stream)['DWAPlannerROS']
        self.assertEqual(common['global_frame'], 'odom')
        self.assertEqual(common['robot_base_frame'], 'base_link')
        self.assertEqual(move_base['base_global_planner'], 'navfn/NavfnROS')
        self.assertEqual(move_base['base_local_planner'],
                         'dwa_local_planner/DWAPlannerROS')
        self.assertFalse(local['holonomic_robot'])

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
        self.assertEqual(scan['topic'], '/scan')
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

    def test_rear_obstacle_probe_covers_blocked_and_clear_costmap_cases(self):
        launch = self.read('launch/rear_obstacle_safety_probe.launch')
        self.assertIn('<arg name="obstacle_y"', launch)
        self.assertIn('<arg name="expect_blocked"', launch)
        self.assertIn('value="$(arg obstacle_y)"', launch)
        self.assertIn('value="$(arg expect_blocked)"', launch)
        probe = self.read('scripts/probe_rear_obstacle_safety.py')
        self.assertIn('OccupancyGrid', probe)
        self.assertIn('obstacle_in_costmap', probe)
        self.assertIn('GoalStatus.SUCCEEDED', probe)
        self.assertIn("'arrived': arrived", probe)
        model = self.read('models/rear_gate_obstacle.sdf')
        self.assertIn('<size>0.20 0.20 1.00</size>', model)

    def test_heading_gate_runtime_dependencies_are_declared(self):
        package = ET.parse(os.path.join(PACKAGE, 'package.xml')).getroot()
        exec_dependencies = [item.text
                             for item in package.findall('exec_depend')]
        self.assertIn('std_msgs', exec_dependencies)
        cmake = self.read('CMakeLists.txt')
        self.assertIn('scripts/heading_gate_node.py', cmake)

    def test_matrix_has_multiple_unique_approach_poses(self):
        with open(os.path.join(PACKAGE, 'config',
                               'navigation_scenarios.yaml')) as stream:
            scenarios = yaml.safe_load(stream)['scenarios']
        self.assertGreaterEqual(len(scenarios), 14)
        poses = set((item['x'], item['y'], item['yaw']) for item in scenarios)
        self.assertEqual(len(poses), len(scenarios))
        self.assertGreaterEqual(sum(bool(item.get('expect_heading_gate'))
                                    for item in scenarios), 3)
        self.assertTrue(any(
            item.get('id') == 'near_rear_reverse_allowed' and
            not item.get('expect_heading_gate') for item in scenarios))

    def test_trial_launch_has_required_monitor_and_no_manipulation(self):
        text = self.read('launch/navigation_trial.launch')
        root = ET.fromstring(text)
        monitor = root.find(".//node[@type='navigation_trial_monitor.py']")
        self.assertIsNotNone(monitor)
        self.assertEqual(monitor.attrib.get('required'), 'true')
        self.assertNotIn('/brick_pose', text)
        self.assertNotIn('pick', text.lower())
        monitor_source = self.read('scripts/navigation_trial_monitor.py')
        self.assertIn('OccupancyGrid', monitor_source)
        self.assertIn('wait_for_fresh_costmap', monitor_source)

    def test_matrix_runner_enforces_full_scenario_set(self):
        text = self.read('scripts/run_navigation_matrix.py')
        self.assertIn('MINIMUM_TRIALS = 14', text)
        self.assertNotIn("add_argument('--limit'", text)
        self.assertIn('container_exit_code', text)

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

    def test_approach_matrix_has_valid_fallback_and_rejection_cases(self):
        with open(os.path.join(PACKAGE, 'config',
                               'approach_scenarios.yaml')) as stream:
            scenarios = yaml.safe_load(stream)['scenarios']
        self.assertGreaterEqual(len(scenarios), 10)
        self.assertGreaterEqual(sum(bool(item['expect_generation'])
                                    for item in scenarios), 8)
        self.assertTrue(any(item.get('small_obstacle') for item in scenarios))
        self.assertTrue(any(not item['expect_generation'] and
                            abs(float(item['brick_x'])) > 6.0
                            for item in scenarios))
        self.assertEqual(len(set(item['id'] for item in scenarios)),
                         len(scenarios))
        launch = self.read('launch/approach_pose_trial.launch')
        self.assertIn('-model approach_obstacle', launch)
        self.assertIn('-y $(arg obstacle_y) -z 0.50', launch)

    def test_approach_runtime_files_are_installed(self):
        cmake = self.read('CMakeLists.txt')
        for name in ('approach_pose_node.py', 'approach_trial_monitor.py',
                     'run_approach_matrix.py'):
            self.assertIn('scripts/' + name, cmake)
        package = ET.parse(os.path.join(PACKAGE, 'package.xml')).getroot()
        dependencies = [item.text for item in package.findall('exec_depend')]
        self.assertIn('visualization_msgs', dependencies)

    def test_approach_matrix_runner_enforces_complete_fresh_trials(self):
        text = self.read('scripts/run_approach_matrix.py')
        self.assertIn('MINIMUM_TRIALS = 10', text)
        self.assertNotIn("add_argument('--limit'", text)
        self.assertIn("result_path.unlink()", text)
        self.assertIn("result['container_exit_code'] = exit_code", text)
        self.assertIn("all(item.get('success')", text)

    def test_docker_image_installs_melodic_navigation(self):
        with open(os.path.join(WORKSPACE, 'docker', 'Dockerfile')) as stream:
            text = stream.read()
        self.assertIn('ros-melodic-navigation', text)


if __name__ == '__main__':
    unittest.main()
