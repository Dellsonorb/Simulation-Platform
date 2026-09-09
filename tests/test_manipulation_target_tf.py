"""Offline target-frame freshness tests with the installed native TF buffer."""

import copy
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest import mock

import yaml


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src/demos/air_ground_pick_demo"
sys.path.insert(0, str(PACKAGE / "src"))
try:
    import rospy
    import tf2_ros
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from moveit_msgs.msg import PlanningScene
    from tf2_geometry_msgs import do_transform_pose
    spec = importlib.util.spec_from_file_location(
        "target_tf_demo", PACKAGE / "scripts/run_air_ground_pick_demo.py")
    demo = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(demo)
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


@unittest.skipUnless(ROS_AVAILABLE, "native ROS message and TF dependencies required")
class ManipulationTargetTfTest(unittest.TestCase):
    def setUp(self):
        self.owner = demo.AirGroundPickDemo.__new__(demo.AirGroundPickDemo)
        config = yaml.safe_load((PACKAGE / "config/demo.yaml").read_text())
        self.owner._ground_tf_max_age = config["ground_tf_max_age"]
        self.owner._target_size = config["target_size"]
        self.owner._scene_robot_links = (
            "ground/base_link", "ground/forearm_link") + demo.manipulation_scene.FINGER_LINKS
        self.owner._tf_buffer = tf2_ros.Buffer(debug=False)
        self.owner._move_group = types.SimpleNamespace(
            get_planning_frame=lambda: "ground/base_link")
        self.applied, self.statuses = [], []
        # These are the external MoveIt transports. The target conversion,
        # scene diff, ROS messages and TF interpolation remain native code.
        self.owner._get_manipulation_scene = lambda: PlanningScene()
        self.owner._apply_manipulation_scene = self.applied.append
        self.owner._publish_status = lambda state, **data: self.statuses.append((state, data))
        self.now, self.wall = 154.669, 100.0
        self.waits = []
        self.after_wait = lambda: None

        def initialize():
            # Recorded MoveIt startup: TF last cached at 154.661, ready after
            # 155.952. Python callbacks resume after the constructor returns.
            self.now = 155.978
            return self.owner._move_group

        self.owner._initialize_moveit = initialize

        def wallsleep(duration):
            self.assertGreater(duration, 0.0)
            self.waits.append(duration)
            self.wall += duration
            self.after_wait()

        patches = (
            mock.patch.object(rospy.Time, "now", side_effect=lambda: rospy.Time.from_sec(self.now)),
            mock.patch.object(demo.time, "monotonic", side_effect=lambda: self.wall),
            mock.patch.object(rospy.rostime, "wallsleep", side_effect=wallsleep),
            mock.patch.object(rospy, "sleep", side_effect=AssertionError("TF wait used simulated time")),
            mock.patch.object(rospy, "is_shutdown", return_value=False),
        )
        for patch in patches:
            patch.start()
            self.addCleanup(patch.stop)
        fixed = TransformStamped()
        fixed.header.frame_id, fixed.child_frame_id = "map", "ground/odom"
        fixed.transform.rotation.w = 1.0
        self.owner._tf_buffer.set_transform_static(fixed, "offline_fixture")
        self.accepted = PoseStamped()
        self.accepted.header.frame_id = "map"
        self.accepted.header.stamp = rospy.Time.from_sec(154.669)
        self.accepted.pose.position.x = 3.2
        self.accepted.pose.position.y = -0.3
        self.accepted.pose.position.z = self.owner._target_size[2] / 2.0
        self.accepted.pose.orientation.w = 1.0
        self.original = copy.deepcopy(self.accepted)

    def insert(self, stamp, x=2.4):
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = "ground/odom", "ground/base_link"
        transform.header.stamp = rospy.Time.from_sec(stamp)
        transform.transform.translation.x = x
        transform.transform.translation.y = -0.67
        transform.transform.rotation.w = 1.0
        self.owner._tf_buffer.set_transform(transform, "offline_fixture")

    def update(self):
        try:
            self.owner._update_manipulation_target(self.accepted, source="accepted_aerial")
        except Exception as error:
            self.fail("target update did not wait for fresh native TF: %s" % error)

    def assert_scene_uses_latest_transform(self):
        latest = self.owner._tf_buffer.lookup_transform(
            "ground/base_link", "map", rospy.Time(0))
        expected = do_transform_pose(self.accepted, latest)
        self.assertEqual(1, len(self.applied))
        target, = self.applied[0].world.collision_objects
        self.assertEqual(expected.header, target.header)
        self.assertEqual(expected.pose, target.primitive_poses[0])
        self.assertEqual(self.owner._target_size, target.primitives[0].dimensions)
        self.assertEqual(self.original, self.accepted)
        self.assertEqual("accepted_aerial", self.statuses[0][1]["source"])
        self.assertFalse(self.statuses[0][1]["target_contacts_allowed"])

    def assert_failed_without_scene(self, reason):
        with self.assertRaisesRegex(demo.DemoError, reason):
            self.owner._update_manipulation_target(self.accepted)
        self.assertEqual([], self.applied)
        self.assertEqual([], self.statuses)
        self.assertEqual(self.original, self.accepted)

    def test_cached_tf_stale_after_moveit_waits_for_new_native_transform(self):
        self.insert(154.661, x=2.0)
        self.after_wait = lambda: self.insert(155.961, x=2.4)
        self.update()
        self.assertTrue(self.waits)
        self.assert_scene_uses_latest_transform()

    def test_existing_fresh_transform_is_used_without_delay(self):
        self.insert(155.961)
        self.update()
        self.assertEqual([], self.waits)
        self.assert_scene_uses_latest_transform()

    def test_missing_transform_can_arrive_during_wall_wait(self):
        self.after_wait = lambda: self.insert(155.961)
        self.update()
        self.assertTrue(self.waits)
        self.assert_scene_uses_latest_transform()

    def test_fresh_arrival_after_delayed_wall_wake_cannot_exceed_budget(self):
        self.insert(154.661)
        def delayed():
            self.wall = 101.
            self.insert(155.961)
        self.after_wait = delayed
        self.assert_failed_without_scene('target planning-frame transform')

    def test_permanently_stale_tf_times_out_with_frozen_sim_time(self):
        self.insert(154.661)
        self.assert_failed_without_scene("target planning-frame transform is stale")
        self.assertGreaterEqual(self.wall - 100.0, 0.5)
        self.assertLessEqual(self.wall - 100.0, 0.55)
        self.assertEqual(155.978, self.now)

    def test_missing_tf_times_out_without_ros_time_sleep(self):
        self.assert_failed_without_scene("target planning-frame transform is unavailable")
        self.assertGreaterEqual(self.wall - 100.0, 0.5)
        self.assertLessEqual(self.wall - 100.0, 0.55)

    def test_new_but_still_stale_transforms_are_never_accepted(self):
        self.insert(154.661)
        self.after_wait = lambda: self.insert(154.661 + self.wall - 100.0)
        self.assert_failed_without_scene("target planning-frame transform is stale")
        self.assertGreaterEqual(self.wall - 100.0, 0.5)

    def test_future_transform_keeps_existing_freshness_rejection(self):
        self.insert(156.2)
        self.assert_failed_without_scene("target planning-frame transform is stale")

    def test_shutdown_prevents_scene_application(self):
        self.insert(155.961)
        with mock.patch.object(rospy, "is_shutdown", return_value=True):
            self.assert_failed_without_scene("shutdown")

    def test_same_frame_needs_no_tf_and_preserves_accepted_pose(self):
        self.accepted.header.frame_id = "ground/base_link"
        self.owner._update_manipulation_target(self.accepted)
        target, = self.applied[0].world.collision_objects
        self.assertEqual(self.accepted.header, target.header)
        self.assertEqual(self.accepted.pose, target.primitive_poses[0])
        self.assertEqual([], self.waits)


if __name__ == "__main__":
    unittest.main()
