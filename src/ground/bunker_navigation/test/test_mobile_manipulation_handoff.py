from __future__ import division

import os
import unittest
import xml.etree.ElementTree as ET


PACKAGE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class MobileManipulationHandoffTest(unittest.TestCase):
    def test_same_model_is_anchored_and_released_without_respawn(self):
        path = os.path.join(PACKAGE, 'src', 'bunker_planar_drive_plugin.cpp')
        with open(path) as stream:
            source = stream.read()
        self.assertIn('HandoffCommand', source)
        self.assertIn('CreateJoint', source)
        self.assertIn('ANCHORED_RELEASED', source)
        self.assertIn('ACTIVE_READY', source)
        self.assertIn('SetKinematic(false)', source)
        self.assertNotIn('DeleteModel', source)
        self.assertNotIn('SpawnModel', source)

    def test_arm_is_a_fixed_payload_until_running_controllers_release_it(self):
        path = os.path.join(PACKAGE, 'src', 'bunker_planar_drive_plugin.cpp')
        with open(path) as stream:
            source = stream.read()
        load = source[source.index('void Load('):source.index(' private:')]
        release = source[source.index('void ReleaseAnchoredManipulator'):
                         source.index('void ActivateManipulatorGravity')]
        self.assertIn('const bool gripper', load)
        self.assertIn('link->SetKinematic(!gripper)', load)
        self.assertIn('gripper_links_.push_back(link)', load)
        self.assertIn('collision->SetCollideBits(0u)', load)
        self.assertIn('TryLockGripperPayload', source)
        self.assertIn('GRIPPER_SETTLING', source)
        self.assertIn('STARTUP_GRIPPER_SETTLED', source)
        self.assertIn('link->SetKinematic(false)', release)
        self.assertIn('collision->SetCollideBits(original->second)', release)

    def test_topics_are_explicit_and_configurable(self):
        urdf = os.path.join(
            os.path.dirname(PACKAGE), 'bunker_aubo_description', 'urdf',
            'bunker_aubo.urdf.xacro')
        with open(urdf) as stream:
            text = stream.read()
        self.assertIn('<handoffCommandTopic>', text)
        self.assertIn('/ground_pick/mobile_manipulation/command', text)
        self.assertIn('<handoffStatusTopic>', text)
        self.assertIn('/ground_pick/mobile_manipulation/status', text)

    def test_navigation_commands_stop_after_release(self):
        path = os.path.join(PACKAGE, 'src', 'bunker_planar_drive_plugin.cpp')
        with open(path) as stream:
            source = stream.read()
        self.assertIn('if (manipulator_released_ || release_failed_)', source)
        self.assertIn('if (!compatibility_initialized_)', source)
        self.assertIn('command = geometry_msgs::Twist();', source)

    def test_plugin_never_initializes_joints_directly(self):
        path = os.path.join(PACKAGE, 'src', 'bunker_planar_drive_plugin.cpp')
        with open(path) as stream:
            source = stream.read()
        self.assertNotIn('SetJointPosition', source)

    def test_spawn_only_startup_pose_is_explicit_and_opt_in(self):
        launch_path = os.path.join(
            os.path.dirname(PACKAGE), 'bunker_aubo_gazebo', 'launch',
            'combined_robot.launch')
        root = ET.parse(launch_path).getroot()
        arguments = {arg.attrib['name']: arg.attrib.get('default')
                     for arg in root.findall('arg')}
        self.assertEqual('false', arguments['mobile_manipulation_startup'])
        startup_spawners = root.findall(
            ".//node[@name='spawn_bunker_aubo_startup_pose']")
        self.assertEqual(1, len(startup_spawners))
        spawner = startup_spawners[0]
        self.assertEqual('$(arg mobile_manipulation_startup)',
                         spawner.attrib['if'])
        spawn_args = spawner.attrib['args']
        for joint in ('shoulder_pan_joint', 'shoulder_lift_joint',
                      'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                      'wrist_3_joint', 'left_outer_knuckle_joint'):
            self.assertIn('-J ' + joint, spawn_args)

    def test_collision_masks_are_restored_not_broadened(self):
        path = os.path.join(PACKAGE, 'src', 'bunker_planar_drive_plugin.cpp')
        with open(path) as stream:
            source = stream.read()
        self.assertIn('dGeomGetCollideBits', source)
        self.assertIn('original_collide_bits_', source)
        self.assertNotIn('SetCollideBits(0xffffu)', source)

    def test_stationary_planar_base_does_not_reteleport_physical_arm(self):
        path = os.path.join(PACKAGE, 'src', 'bunker_planar_drive_plugin.cpp')
        with open(path) as stream:
            source = stream.read()
        self.assertNotIn('if (!manipulator_released_ || moving)', source)
        self.assertIn('if (moving) {', source)


if __name__ == '__main__':
    unittest.main()
