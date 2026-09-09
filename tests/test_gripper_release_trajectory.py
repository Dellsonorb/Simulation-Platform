"""Offline action-goal regression for opening after a confirmed contact stall."""

import copy
import importlib.util
from pathlib import Path
import sys
import threading
import types
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src/demos/air_ground_pick_demo"
sys.path.insert(0, str(PACKAGE / "src"))
try:
    import rospy
    from sensor_msgs.msg import JointState
    spec = importlib.util.spec_from_file_location(
        "gripper_release_demo", PACKAGE / "scripts/run_air_ground_pick_demo.py")
    demo = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(demo)
    ROS_MESSAGES_AVAILABLE = True
except ImportError:
    ROS_MESSAGES_AVAILABLE = False


@unittest.skipUnless(ROS_MESSAGES_AVAILABLE, "native ROS message dependencies required")
class GripperReleaseTrajectoryTest(unittest.TestCase):
    def setUp(self):
        self.owner = demo.AirGroundPickDemo.__new__(demo.AirGroundPickDemo)
        owner = self.owner
        owner._lock = threading.RLock()
        owner._ground_target_pose = None
        owner._ground_pose_received = None
        owner._grasp_confirmed = False
        owner._grasp_confirmation_received = None
        owner._joint_state = JointState()
        owner._joint_state.header.stamp = rospy.Time.from_sec(49.05)
        owner._joint_state.name = ["wrist_3_joint", "left_outer_knuckle_joint"]
        owner._joint_state.position = [2.0, 0.51685]
        owner._joint_state.velocity = [0.2, -0.003]
        owner._joint_state_received = 100.0
        owner._gripper_open_position = 0.0
        owner._gripper_motion_time = 2.0
        owner._gripper_action_timeout = 8.0
        owner._gripper_joint_tolerance = 0.10
        owner._maximum_gripper_opening = 0.0952
        owner._maximum_gripper_joint = 0.93
        owner._feasibility = types.SimpleNamespace(required_opening=0.055)
        self.goals = []

        def wait_result(_timeout):
            # The real controller is the only unavailable side effect. Feed
            # its successful measured endpoint to the real aperture checks.
            owner._joint_state.name = ["wrist_3_joint", "left_outer_knuckle_joint"]
            owner._joint_state.position = [2.0, 0.0]
            owner._joint_state.velocity = [0.2, 0.0]
            owner._joint_state_received = 100.0
            return True

        owner._gripper_client = types.SimpleNamespace(
            wait_for_server=lambda timeout: True,
            send_goal=lambda goal: self.goals.append(copy.deepcopy(goal)),
            wait_for_result=wait_result,
            get_state=lambda: demo.GoalStatus.SUCCEEDED)
        self.patches = [
            mock.patch.object(demo.rospy.Time, "now", return_value=rospy.Time.from_sec(49.05)),
            mock.patch.object(demo.rospy, "is_shutdown", return_value=False),
            mock.patch.object(demo.time, "monotonic", return_value=100.0),
        ]
        for patch in self.patches:
            patch.start()
            self.addCleanup(patch.stop)

    @staticmethod
    def points(goal):
        return [(list(point.positions), list(point.velocities), point.time_from_start.to_sec())
                for point in goal.trajectory.points]

    def test_open_after_stall_starts_at_measured_position_and_velocity(self):
        original = self.owner._trajectory_goal(
            ("left_outer_knuckle_joint",), (0.0,), 2.0)
        self.assertEqual([([0.0], [0.0], 2.0)], self.points(original))
        # With no explicit start, the controller bridges from its stale
        # closure target .70000; the actual .51685 exceeds the .08 path limit.
        self.assertAlmostEqual(0.18315, 0.70000-0.51685)
        self.assertGreater(0.70000-0.51685, 0.08)
        opening = self.owner._open_gripper()
        revised = self.goals[0]
        self.assertEqual(
            [([0.51685], [-0.003], 0.0), ([0.0], [0.0], 2.0)],
            self.points(revised))
        self.assertEqual(original.trajectory.header, revised.trajectory.header)
        self.assertEqual(original.trajectory.joint_names, revised.trajectory.joint_names)
        self.assertEqual(original.path_tolerance, revised.path_tolerance)
        self.assertEqual(original.goal_tolerance, revised.goal_tolerance)
        self.assertEqual(original.goal_time_tolerance, revised.goal_time_tolerance)
        self.assertAlmostEqual(0.0952, opening)

    def test_other_trajectory_commands_keep_original_single_endpoint(self):
        self.owner._execute_trajectory(
            self.owner._gripper_client, ("left_outer_knuckle_joint",),
            (0.7,), 2.0, 8.0, "AG95 close", accept_grasp_stall=True)
        self.assertEqual([([0.7], [0.0], 2.0)], self.points(self.goals[0]))

    def test_open_rejects_stale_measured_start_without_sending(self):
        self.owner._joint_state_received = 98.0
        with self.assertRaisesRegex(demo.DemoError, "stale"):
            self.owner._open_gripper()
        self.assertEqual([], self.goals)

    def test_open_rejects_missing_or_nonfinite_measured_start(self):
        message = self.owner._joint_state
        for position, velocity, names in (
                ([2., float("nan")], [.2, 0.], message.name),
                ([2., .51685], [.2, float("nan")], message.name),
                ([2., .51685], [], message.name),
                ([2., .51685], [.2, 0.], ["wrist_3_joint", "wrong_joint"])):
            with self.subTest(position=position, velocity=velocity, names=names):
                message.position = list(position)
                message.velocity = list(velocity)
                message.name = list(names)
                with self.assertRaises(demo.DemoError):
                    self.owner._open_gripper()
                self.assertEqual([], self.goals)

    def test_open_preserves_strict_measured_aperture_gate(self):
        self.owner._feasibility.required_opening = self.owner._maximum_gripper_opening
        with self.assertRaisesRegex(demo.DemoError, "opening is too small"):
            self.owner._open_gripper()
        self.assertEqual(1, len(self.goals))


if __name__ == "__main__":
    unittest.main()
