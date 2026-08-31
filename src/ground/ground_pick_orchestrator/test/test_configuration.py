from __future__ import division

import os
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class ConfigurationTest(unittest.TestCase):
    def test_demo_composes_existing_modules_without_duplicate_robot(self):
        path = os.path.join(PACKAGE, 'launch', 'ground_pick_demo.launch')
        root = ET.parse(path).getroot()
        includes = root.findall('include')
        files = [item.get('file') for item in includes]
        combined = [item for item in includes
                    if 'combined_robot.launch' in item.get('file', '')]
        self.assertEqual(1, len(combined))
        arguments = {item.get('name'): item.get('value')
                     for item in combined[0].findall('arg')}
        self.assertEqual('true', arguments['camera_enable_rgbd'])
        self.assertEqual('true', arguments['mobile_base'])
        self.assertEqual('true', arguments['navigation_laser'])
        self.assertEqual('true', arguments['spawn_controllers'])
        self.assertEqual('true', arguments['mobile_manipulation_startup'])
        text = open(path).read()
        self.assertNotIn('visual_pick_demo.launch', text)
        self.assertNotIn('approach_pose_demo.launch', text)
        self.assertNotIn('manual_brick_pose_publisher', text)
        self.assertIn('moveit_planning_execution.launch', text)
        self.assertIn('perception.launch', text)
        self.assertIn('pose_estimator.launch', text)
        self.assertIn('approach_pose_node.py', text)
        self.assertIn('heading_gate_node.py', text)
        self.assertIn('move_base', text)

    def test_pick_node_only_consumes_approved_private_pose(self):
        path = os.path.join(PACKAGE, 'launch', 'ground_pick_demo.launch')
        root = ET.parse(path).getroot()
        pick = root.find(".//node[@name='brick_pick_node']")
        self.assertIsNotNone(pick)
        remaps = {item.get('from'): item.get('to')
                  for item in pick.findall('remap')}
        self.assertEqual('/ground_pick/approved_brick_pose',
                         remaps['/brick_pose'])

    def test_approach_matches_validated_v02_observation_distance(self):
        root = ET.parse(os.path.join(
            PACKAGE, 'launch', 'ground_pick_demo.launch')).getroot()
        node = root.find(".//node[@name='approach_pose_generator']")
        params = {item.get('name'): item.get('value')
                  for item in node.findall('param')}
        self.assertEqual('$(arg visual_work_distance)',
                         params['work_distance'])
        self.assertLess(float(params['reach_min']), 0.67)

    def test_mission_runtime_uses_trajectory_without_gazebo_teleport(self):
        source = open(os.path.join(
            PACKAGE, 'scripts', 'ground_pick_node.py')).read()
        self.assertNotIn("rospy.delete_param('/gazebo_ros_control/pid_gains')",
                         source)
        self.assertNotIn("stopped_velocity_tolerance', -1.0", source)
        self.assertNotIn("left_outer_knuckle_joint/trajectory', -1.0", source)
        for forbidden in (
                'SetModelConfiguration', 'SpawnModel',
                "'/gazebo/delete_model'", "'/gazebo/spawn_urdf_model'",
                'set_model_configuration', 'respawn_manipulation_model'):
            self.assertNotIn(forbidden, source)
        self.assertIn('FollowJointTrajectoryGoal', source)
        self.assertNotIn('JointTrajectoryPoint', source)
        self.assertIn('MoveGroupCommander', source)
        self.assertIn("add_box(\n                    'ground_pick_forward_ground_guard'", source)
        self.assertIn('validated_observation_trajectory', source)
        self.assertNotIn('clear_joint_value_targets', source)
        self.assertIn('self.arm_controller.send_goal(goal)', source)
        self.assertIn("String(data='RELEASE')", source)
        self.assertIn("String(data='ENABLE_GRAVITY')", source)

    def test_refine_gate_is_configured_for_staging_not_formal_pose(self):
        path = os.path.join(PACKAGE, 'config', 'mission.yaml')
        config = yaml.safe_load(open(path))
        self.assertEqual('/ground_pick/refined_brick_pose',
                         config['refine_output_topic'])
        self.assertEqual('/brick_pose', config['formal_pose_topic'])
        self.assertEqual('/ground_pick/approved_brick_pose',
                         config['approved_pose_topic'])
        self.assertEqual('aubo_i5_base_link',
                         config['required_pose_frame'])
        self.assertGreaterEqual(config['stop_settle_time'], 1.0)
        self.assertEqual(5.0, config['observation_duration'])

    def test_launch_does_not_provide_respawn_only_description(self):
        text = open(os.path.join(
            PACKAGE, 'launch', 'ground_pick_demo.launch')).read()
        self.assertNotIn('ground_pick_manipulation_description', text)

    def test_model_identity_is_preserved_through_manipulation(self):
        source = open(os.path.join(
            PACKAGE, 'scripts', 'ground_pick_node.py')).read()
        self.assertNotIn('/gazebo/delete_model', source)
        self.assertNotIn('/gazebo/spawn_urdf_model', source)
        self.assertIn('wait_mobile_manipulation_state', source)

    def test_matrix_has_ten_valid_and_three_failure_controls(self):
        path = os.path.join(PACKAGE, 'config', 'mission_scenarios.yaml')
        config = yaml.safe_load(open(path))
        valid = [item for item in config['scenarios']
                 if item['expect_success']]
        controls = [item for item in config['scenarios']
                    if not item['expect_success']]
        self.assertGreaterEqual(len(valid), 10)
        self.assertEqual(
            {'navigation_failed', 'perception_rejected', 'planning_failed'},
            {item['expected_failure'] for item in controls})
        self.assertEqual(len(config['scenarios']),
                         len({item['id'] for item in config['scenarios']}))
        for relative in ('scripts/ground_pick_trial_monitor.py',
                         'scripts/run_ground_pick_matrix.py'):
            self.assertTrue(os.access(os.path.join(PACKAGE, relative), os.X_OK))
        ET.parse(os.path.join(PACKAGE, 'launch', 'ground_pick_trial.launch'))

    def test_orchestrator_does_not_reimplement_perception_or_grasp(self):
        path = os.path.join(PACKAGE, 'scripts', 'ground_pick_node.py')
        self.assertTrue(os.access(path, os.X_OK))
        # A stale module with the package name shadows the installed Python
        # package when roslaunch executes this script from its source folder.
        self.assertFalse(os.path.exists(os.path.join(
            PACKAGE, 'scripts', 'ground_pick_orchestrator.py')))
        self.assertFalse(os.path.exists(os.path.join(
            PACKAGE, 'scripts', 'ground_pick_orchestrator.pyc')))
        text = open(path).read()
        self.assertNotIn('ApproximateTimeSynchronizer', text)
        self.assertNotIn('PointCloud2', text)
        self.assertNotIn('MoveGroupInterface', text)
        self.assertNotIn('generateTopDownGrasp', text)
        self.assertIn('self.request_manipulation_controllers()', text)
        self.assertIn('self.wait_manipulation_controllers()', text)
        start_refine = text[text.index('    def refine_worker_main(self):'):
                            text.index('    def request_manipulation_controllers')]
        self.assertLess(start_refine.index(
            'self.prepare_mobile_manipulation()'),
            start_refine.index('self.move_to_observation_trajectory()'))
        self.assertLess(start_refine.index(
            'self.move_to_observation_trajectory()'),
            start_refine.index('self.activate_manipulator_gravity()'))
        self.assertNotIn('self.hold_gripper_open()', start_refine)
        approve = text[text.index('    def manipulation_worker_main(self'):
                       text.index('    def refine_status_callback')]
        self.assertNotIn('respawn', approve)
        self.assertIn('self.formal_pose_pub.publish(message)', approve)

    def test_blocking_gazebo_handoffs_are_daemon_workers(self):
        source = open(os.path.join(
            PACKAGE, 'scripts', 'ground_pick_node.py')).read()
        self.assertIn('def start_daemon_worker(', source)
        self.assertIn("name='ground_pick_refine'", source)
        self.assertIn("name='ground_pick_manipulation'", source)
        self.assertIn("name='ground_pick_lift_verification'", source)
        timer = source[source.index('    def timer_callback(self, _event):'):
                       source.index('    def safety_stop(self):')]
        self.assertNotIn('self.brick_height()', timer)
        self.assertNotIn('self.respawn_manipulation_model()', timer)
        callback = source[source.index('    def refined_pose_callback'):
                          source.index('    def begin_manipulation_handoff')]
        self.assertNotIn('self.respawn_manipulation_model()', callback)
        safety = source[source.index('    def safety_stop(self):'):
                        source.index('    def stop_children')]
        self.assertNotIn('self.unpause_physics()', safety)
        self.assertIn('self.zero_command_pub.publish(Twist())', safety)

    def test_handoff_commands_are_bounded_and_fail_closed(self):
        source = open(os.path.join(
            PACKAGE, 'scripts', 'ground_pick_node.py')).read()
        self.assertIn('self.mobile_handoff_timeout', source)
        self.assertIn("wait_mobile_manipulation_state('ANCHORED_RELEASED'",
                      source)
        self.assertIn("wait_mobile_manipulation_state('ACTIVE_READY'", source)

    def test_trial_monitor_requires_fresh_command_and_physical_stop(self):
        source = open(os.path.join(
            PACKAGE, 'scripts', 'ground_pick_trial_monitor.py')).read()
        self.assertIn('self.latest_cmd = None', source)
        self.assertIn('self.latest_cmd_wall = None', source)
        self.assertIn('self.terminal_wall = None', source)
        self.assertIn('terminal_stop_is_safe(', source)
        self.assertIn("'stop_command_fresh'", source)
        self.assertIn("'model_stationary'", source)

    def test_manifest_declares_direct_launch_and_python_dependencies(self):
        root = ET.parse(os.path.join(PACKAGE, 'package.xml')).getroot()
        dependencies = {item.text for item in root.findall('exec_depend')}
        self.assertTrue({
            'bunker_aubo_description', 'gazebo_ros', 'move_base', 'rviz',
            'moveit_commander', 'xacro', 'python3-yaml'}.issubset(dependencies))


if __name__ == '__main__':
    unittest.main()
