#!/usr/bin/env python3
"""Ground-only geometric acceptance and raw aiming-cue regressions."""

from collections import deque
import importlib.util
import math
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src/demos/air_ground_pick_demo"
SPEC = importlib.util.spec_from_file_location(
    "ground_top_perception", PACKAGE / "src/air_ground_pick_demo/perception.py")
PERCEPTION = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PERCEPTION)


def top_patch(length=0.240, width=0.053, yaw=0.38, z=0.115):
    along, across = np.meshgrid(
        np.linspace(-length / 2, length / 2, 41),
        np.linspace(-width / 2, width / 2, 17))
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack((
        1.7 + c * along.ravel() - s * across.ravel(),
        -0.2 + s * along.ravel() + c * across.ravel(),
        np.full(along.size, z)))


def side_patch():
    along, vertical = np.meshgrid(
        np.linspace(-0.12, 0.12, 41), np.linspace(0.002, 0.093, 31))
    yaw = -0.5313
    return np.column_stack((
        1.88 + math.cos(yaw) * along.ravel(),
        -0.24 + math.sin(yaw) * along.ravel(), vertical.ravel()))


class GroundTopSurfaceTest(unittest.TestCase):
    def strict(self, points, **options):
        try:
            return PERCEPTION.estimate_target_pose(
                points, 0.115, 0.015,
                target_top_size=(0.240, 0.053), **options)
        except TypeError as error:
            self.fail("Ground top acceptance option is missing: %s" % error)

    def test_vertical_side_is_rejected_in_ground_mode(self):
        with self.assertRaisesRegex(PERCEPTION.PerceptionError, "horizontal"):
            self.strict(side_patch())

    def test_incomplete_long_or_short_span_is_rejected(self):
        for length, width in ((0.20, 0.053), (0.240, 0.040)):
            with self.subTest(length=length, width=width):
                with self.assertRaisesRegex(PERCEPTION.PerceptionError, "span"):
                    self.strict(top_patch(length=length, width=width))

    def test_oversized_surface_is_not_the_known_target_top(self):
        with self.assertRaisesRegex(PERCEPTION.PerceptionError, "span"):
            self.strict(top_patch(length=0.30))

    def test_complete_rotated_top_preserves_measured_center_and_yaw(self):
        for yaw in (-1.2, 0.0, 0.38, 1.4):
            with self.subTest(yaw=yaw):
                pose = self.strict(top_patch(yaw=yaw, z=0.123))
                np.testing.assert_allclose(pose[:3], [1.7, -0.2, 0.0655], atol=1e-12)
                self.assertLess(PERCEPTION.yaw_error_mod_pi(pose[3], yaw), 1e-10)

    def test_ninety_percent_spans_are_accepted_without_nominal_center_injection(self):
        pose = self.strict(top_patch(length=0.216, width=0.0477, z=0.15))
        np.testing.assert_allclose(pose[:3], [1.7, -0.2, 0.0925], atol=1e-12)

    def test_measured_top_above_tilt_limit_is_rejected(self):
        points = top_patch(yaw=0.0)
        points[:, 2] += math.tan(math.radians(16.0)) * (points[:, 1] + 0.2)
        with self.assertRaisesRegex(PERCEPTION.PerceptionError, "horizontal"):
            self.strict(points)

    def test_line_support_is_degenerate(self):
        with self.assertRaisesRegex(PERCEPTION.PerceptionError, "degenerate"):
            self.strict(top_patch(width=0.0))

    def test_default_estimator_keeps_legacy_side_behavior(self):
        points = side_patch()
        pose = PERCEPTION.estimate_target_pose(points, 0.115, 0.015)
        highest = points[points[:, 2] >= np.percentile(points[:, 2], 95) - 0.015]
        self.assertAlmostEqual(np.median(highest[:, 2]) - 0.0575, pose[2])
        self.assertLess(pose[2], 0.03)

    def test_invalid_strict_parameters_are_rejected(self):
        for options in ({"minimum_top_span_fraction": 0.0},
                        {"minimum_top_span_fraction": 1.1},
                        {"maximum_top_tilt_degrees": float("nan")},
                        {"maximum_top_tilt_degrees": 90.0}):
            with self.subTest(options=options):
                with self.assertRaises(PERCEPTION.PerceptionError):
                    self.strict(top_patch(), **options)

    def test_invalid_known_top_dimensions_are_rejected(self):
        for size in ([0.24], [0.24, 0.053, 0.115], [0.053, 0.24],
                     [0.24, 0.0], [float("nan"), 0.053]):
            with self.subTest(size=size):
                with self.assertRaises(PERCEPTION.PerceptionError):
                    PERCEPTION.estimate_target_pose(
                        top_patch(), 0.115, target_top_size=size)

    def test_ground_config_enables_gate_and_cue_without_changing_air(self):
        ground = yaml.safe_load((PACKAGE / "config/ground_observer.yaml").read_text())
        air = yaml.safe_load((PACKAGE / "config/air_observer.yaml").read_text())
        self.assertEqual([0.240, 0.053], ground.get("target_top_size"))
        self.assertEqual(0.9, ground.get("minimum_top_span_fraction"))
        self.assertEqual(15.0, ground.get("maximum_top_tilt_degrees"))
        self.assertEqual("/ground_observer/surface_cue", ground.get("surface_cue_topic"))
        self.assertNotIn("target_top_size", air)
        self.assertNotIn("surface_cue_topic", air)
        self.assertEqual(0.115, ground["target_height"])


try:
    import rospy
    from cv_bridge import CvBridge
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import CameraInfo
    import air_ground_pick_demo.perception
    ROS_IMPORTS = True
except ImportError:
    ROS_IMPORTS = False


@unittest.skipUnless(ROS_IMPORTS, "native ROS message imports are unavailable")
class GroundSurfaceCueObserverTest(unittest.TestCase):
    def setUp(self):
        spec = importlib.util.spec_from_file_location(
            "ground_top_observer", PACKAGE / "scripts/red_target_observer.py")
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.observer = self.module.RedTargetObserver.__new__(self.module.RedTargetObserver)
        self.cues, self.poses, self.status = [], [], []
        values = dict(
            target_frame="map", camera_optical_frame="ground/color", target_height=0.115,
            top_surface_tolerance=0.015, target_top_size=(0.240, 0.053),
            minimum_top_span_fraction=0.9, maximum_top_tilt_degrees=15.0,
            min_depth=0.12, max_depth=2.5, min_pixels=40, minimum_points=40,
            ambiguity_ratio=0.65, max_observation_age=1.0, max_future_skew=0.1,
            stable_frames=4, max_position_spread=0.05, max_yaw_spread=0.2,
            samples=deque(maxlen=4), samples_lock=threading.Lock(), bridge=CvBridge(),
            surface_cue_publisher=SimpleNamespace(publish=self.cues.append),
            pose_publisher=SimpleNamespace(publish=self.poses.append),
            status_publisher=SimpleNamespace(publish=self.status.append))
        for key, value in values.items():
            setattr(self.observer, key, value)
        self.stamp = rospy.Time.from_sec(10.0)
        rgb = np.zeros((9, 25, 3), dtype=np.uint8)
        rgb[:, :, 0] = 230
        self.color = self.observer.bridge.cv2_to_imgmsg(rgb, encoding="rgb8")
        self.depth = self.observer.bridge.cv2_to_imgmsg(
            np.full((9, 25), 0.5, dtype=np.float32), encoding="32FC1")
        self.infos = []
        for message, frame in ((self.color, "ground/color"), (self.depth, "ground/depth")):
            message.header.stamp = self.stamp
            message.header.frame_id = frame
            info = CameraInfo()
            info.header = message.header
            info.height, info.width = 9, 25
            info.K = [100.0, 0, 12.0, 0, 100.0, 4.0, 0, 0, 1]
            self.infos.append(info)
        def lookup(target, source, stamp):
            result = TransformStamped()
            result.header.stamp, result.header.frame_id = stamp, target
            result.child_frame_id = source
            result.transform.rotation.w = 1.0
            if target == "map":
                result.transform.rotation.x, result.transform.rotation.w = 1.0, 0.0
                result.transform.translation.x = 2.0
                result.transform.translation.y = 3.0
                result.transform.translation.z = 0.615
            return result
        self.observer.lookup_transform = lookup
        self.patches = [mock.patch.object(self.module.rospy, "is_shutdown", return_value=False),
                        mock.patch.object(self.module.rospy.Time, "now", return_value=self.stamp),
                        mock.patch.object(self.module.rospy, "logwarn_throttle")]
        for patch in self.patches:
            patch.start()
            self.addCleanup(patch.stop)

    def test_partial_top_publishes_stamped_map_surface_cue_but_no_target_pose(self):
        self.observer.samples.append(np.array([0, 0, 0.0575, 0]))
        self.observer.observe(self.color, self.depth, *self.infos)
        self.assertEqual(1, len(self.cues))
        self.assertEqual("map", self.cues[0].header.frame_id)
        self.assertEqual(self.stamp, self.cues[0].header.stamp)
        point = self.cues[0].point
        np.testing.assert_allclose([point.x, point.y, point.z], [2.0, 3.0, 0.115])
        self.assertEqual([], self.poses)
        self.assertEqual(0, len(self.observer.samples))

    def test_wrong_stream_frame_rejects_before_cue_publication(self):
        self.color.header.frame_id = "wrong_camera"
        self.observer.observe(self.color, self.depth, *self.infos)
        self.assertEqual([], self.cues)
        self.assertEqual([], self.poses)

    def test_image_clipped_top_rejects_pose_even_when_spans_match(self):
        for info in self.infos:
            info.K[0], info.K[4] = 50.0, 4.0 / 0.053
        for _ in range(4):
            self.observer.observe(self.color, self.depth, *self.infos)
        self.assertEqual(4, len(self.cues))
        self.assertEqual([], self.poses)
        self.assertEqual(0, len(self.observer.samples))
        self.assertIn("clipped", self.status[-1].data)

    def test_disabled_ground_options_preserve_default_observer_behavior(self):
        self.observer.target_top_size = None
        self.observer.surface_cue_publisher = None
        for _ in range(4):
            self.observer.observe(self.color, self.depth, *self.infos)
        self.assertEqual([], self.cues)
        self.assertEqual(1, len(self.poses))

    def test_stale_image_rejects_before_cue_publication(self):
        with mock.patch.object(self.module.rospy.Time, "now",
                               return_value=rospy.Time.from_sec(11.1)):
            self.observer.observe(self.color, self.depth, *self.infos)
        self.assertEqual([], self.cues)
        self.assertEqual([], self.poses)

    def test_surface_cue_uses_only_finite_measured_points(self):
        self.observer.publish_surface_cue(
            [[2.0, 3.0, 0.05], [4.0, 5.0, 0.09], [float("nan"), 0, 0]],
            self.stamp)
        point = self.cues[0].point
        np.testing.assert_allclose([point.x, point.y, point.z], [3.0, 4.0, 0.07])

    def test_complete_interior_top_can_publish_ground_pose(self):
        rgb = np.zeros((11, 27, 3), dtype=np.uint8)
        rgb[1:-1, 1:-1, 0] = 230
        color = self.observer.bridge.cv2_to_imgmsg(rgb, encoding="rgb8")
        depth = self.observer.bridge.cv2_to_imgmsg(
            np.full((11, 27), 0.5, dtype=np.float32), encoding="32FC1")
        color.header, depth.header = self.color.header, self.depth.header
        for info in self.infos:
            info.height, info.width = 11, 27
            info.K = [50.0, 0, 13.0, 0, 4.0 / 0.053, 5.0, 0, 0, 1]
        for _ in range(4):
            self.observer.observe(color, depth, *self.infos)
        self.assertEqual(4, len(self.cues))
        self.assertEqual(1, len(self.poses))
        point = self.poses[0].pose.position
        np.testing.assert_allclose([point.x, point.y, point.z], [2.0, 3.0, 0.0575])

    def test_surface_cue_rejects_non_map_output_frame(self):
        self.observer.target_frame = "camera"
        with self.assertRaisesRegex(self.module.PerceptionError, "map frame"):
            self.observer.publish_surface_cue([[1, 2, 3]], self.stamp)
        self.assertEqual([], self.cues)

    def test_nonfinite_transformed_points_do_not_publish_cue(self):
        original = self.observer.lookup_transform
        def invalid(target, source, stamp):
            result = original(target, source, stamp)
            if target == "map":
                result.transform.translation.x = float("nan")
            return result
        self.observer.lookup_transform = invalid
        self.observer.observe(self.color, self.depth, *self.infos)
        self.assertEqual([], self.cues)
        self.assertEqual([], self.poses)


if __name__ == "__main__":
    unittest.main()
