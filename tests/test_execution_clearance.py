"""Native-message tests of the bounded chassis-clearance planning revision."""
import copy
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/demos/air_ground_pick_demo/src'))
try:
    import rospy
    from moveit_msgs.msg import PlanningScene, RobotState, RobotTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    NATIVE = True
except ImportError:
    NATIVE = False

if NATIVE:
    from air_ground_pick_demo import execution_clearance as ec

URDF = '''<robot name="test">
<link name="ground/base_link"><collision><origin xyz=".02 0 -.16" rpy="0 0 .1"/>
<geometry><box size="1 .8 .4"/></geometry></collision></link>
<link name="mount"/><link name="arm"/><link name="ground/d435"/>
<joint name="base_mount" type="fixed"><parent link="ground/base_link"/><child link="mount"/></joint>
<joint name="shoulder" type="revolute"><parent link="mount"/><child link="arm"/></joint>
<joint name="camera" type="fixed"><parent link="arm"/><child link="ground/d435"/></joint>
</robot>'''


@unittest.skipUnless(NATIVE, 'requires native ROS messages and execution module')
class ExecutionClearanceTests(unittest.TestCase):
    def test_prospective_payload_is_distinct_from_real_grasp_confirmation(self):
        from air_ground_pick_demo import manipulation_scene as ms
        from geometry_msgs.msg import PoseStamped
        links = ['ground/base_link', 'tcp'] + list(ms.FINGER_LINKS)
        pose = PoseStamped(); pose.header.frame_id = 'ground/base_link'
        pose.pose.orientation.w = 1.; pose.pose.position.z = .1
        scene = PlanningScene()
        change = ms.world_target_diff(scene, pose, [.24, .053, .115], links)
        scene.world = change.world; scene.allowed_collision_matrix = change.allowed_collision_matrix
        with self.assertRaises(ms.SceneError):
            ms.attach_target_diff(scene, pose, 'tcp', links, False)
        before = copy.deepcopy(scene)
        predicted = ec.predicted_payload_diff(scene, pose, 'tcp', links)
        self.assertEqual(len(predicted.robot_state.attached_collision_objects), 1)
        self.assertEqual(predicted.robot_state.attached_collision_objects[0].object.header.frame_id, 'tcp')
        self.assertEqual(scene, before)

    def test_chassis_copy_dilates_real_box_and_exempts_only_root_fixed_chain(self):
        from air_ground_pick_demo import manipulation_scene as ms
        links = ['ground/base_link', 'mount', 'arm', 'ground/d435'] + list(ms.FINGER_LINKS)
        current = PlanningScene()
        before = copy.deepcopy(current)
        change = ec.chassis_clearance_diff(current, URDF, links)
        obj = change.world.collision_objects[0]
        self.assertEqual(obj.id, ec.CHASSIS_GUARD_ID)
        self.assertEqual(obj.header.frame_id, 'ground/base_link')
        self.assertEqual(obj.primitives[0].dimensions, [1.024, .8240000000000001, .42400000000000004])
        self.assertAlmostEqual(obj.primitive_poses[0].position.x, .02)
        self.assertAlmostEqual(obj.primitive_poses[0].orientation.z, math.sin(.05))
        names = change.allowed_collision_matrix.entry_names
        row = change.allowed_collision_matrix.entry_values[names.index(ec.CHASSIS_GUARD_ID)].enabled
        self.assertTrue(row[names.index('ground/base_link')])
        self.assertTrue(row[names.index('mount')])
        self.assertFalse(row[names.index('arm')])
        self.assertFalse(row[names.index('ground/d435')])
        self.assertFalse(row[names.index(ms.FINGER_LINKS[0])])
        self.assertFalse(row[names.index(ms.TARGET_ID)])
        self.assertEqual(current, before)

    def test_missing_or_nonbox_chassis_is_not_silently_approximated(self):
        for xml in ('<robot name="empty"/>', URDF.replace('<box size="1 .8 .4"/>', '<sphere radius=".4"/>')):
            with self.assertRaises(ValueError):
                ec.chassis_geometry(xml)

    def test_loaded_gripper_range_is_not_controller_force_goal(self):
        values = ec.closure_positions(.29, .457374407, .005)
        self.assertEqual(values[0], .29)
        self.assertAlmostEqual(values[-1], .522)
        self.assertGreater(len(values), 2)
        self.assertLess(max(values), .70)
        with self.assertRaises(ValueError):
            ec.closure_positions(.6, .457374407, .005)

    def test_full_start_state_cannot_truncate_or_invent_nonarm_values(self):
        state = RobotState()
        state.joint_state.name = ['j1', 'gripper']; state.joint_state.position = [.4]
        with self.assertRaises(ValueError):
            ec.state_at_positions(state, ['j1'], [.5])
        state.joint_state.position = [.4, math.nan]
        with self.assertRaises(ValueError):
            ec.state_at_positions(state, ['j1'], [.5])
        state.joint_state.position = [.4, .3]
        with self.assertRaises(ValueError):
            ec.state_at_positions(state, [], [])

    def test_controller_sampler_preserves_quintic_overshoot_and_nonarm_state(self):
        state = RobotState()
        state.joint_state.name = ['j1', 'left_outer_knuckle_joint']
        state.joint_state.position = [0., .3]
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = ['j1']
        trajectory.joint_trajectory.points = [
            JointTrajectoryPoint(positions=[0.], velocities=[1.], accelerations=[0.], time_from_start=rospy.Duration(0.)),
            JointTrajectoryPoint(positions=[0.], velocities=[-1.], accelerations=[0.], time_from_start=rospy.Duration(1.))]
        samples = list(ec.controller_samples(state, trajectory, period_s=.1))
        self.assertGreater(max(s.joint_state.position[0] for s in samples), .30)
        self.assertTrue(all(s.joint_state.position[1] == .3 for s in samples))
        self.assertEqual(samples[-1].joint_state.position[0], 0.)
        self.assertEqual(state.joint_state.position, [0., .3])

    def test_controller_sampler_does_not_accept_zero_time_geometry_as_executed_path(self):
        state = RobotState()
        state.joint_state.name = ['j1']; state.joint_state.position = [0.]
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = ['j1']
        trajectory.joint_trajectory.points = [JointTrajectoryPoint(positions=[0.]), JointTrajectoryPoint(positions=[1.])]
        with self.assertRaises(ValueError):
            list(ec.controller_samples(state, trajectory))

    def test_live_held_desired_bridges_zero_time_first_point_that_jtc_drops(self):
        state = RobotState()
        state.joint_state.name = ['j1']; state.joint_state.position = [.1]
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = ['j1']
        trajectory.joint_trajectory.points = [
            JointTrajectoryPoint(positions=[0.], velocities=[0.], accelerations=[0.]),
            JointTrajectoryPoint(positions=[.2], velocities=[0.], accelerations=[0.], time_from_start=rospy.Duration(1.))]
        hold = JointTrajectoryPoint(positions=[.08], velocities=[0.], accelerations=[0.])
        samples = list(ec.controller_samples(state, trajectory, period_s=.1, held_desired=hold))
        self.assertEqual(samples[0].joint_state.position, [.08])
        self.assertAlmostEqual(samples[5].joint_state.position[0], .14)
        trajectory.joint_trajectory.points.pop(0)
        self.assertEqual(samples, list(ec.controller_samples(state, trajectory, period_s=.1, held_desired=hold)))
        hold.velocities = [.1]
        with self.assertRaises(ValueError):
            list(ec.controller_samples(state, trajectory, held_desired=hold))
