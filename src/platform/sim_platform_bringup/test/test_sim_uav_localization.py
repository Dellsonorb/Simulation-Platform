#!/usr/bin/env python3
"""Offline geometry and same-time SIM localization checks."""

import importlib.util
import io
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "sim_uav_localization.py"
SPEC = importlib.util.spec_from_file_location("sim_uav_localization", SCRIPT)
localization = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(localization)

try:
    import rospy
    import tf2_ros
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
except ImportError:
    rospy = tf2_ros = TransformStamped = None


def rigid(translation=(0.0, 0.0, 0.0), angles=(0.0, 0.0, 0.0)):
    """Independent roll/pitch/yaw fixture, using radians."""
    roll, pitch, yaw = angles
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    result = np.eye(4)
    result[:3, :3] = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])
    result[:3, 3] = translation
    return result


class MapToOdomTest(unittest.TestCase):
    def test_preserves_nonzero_physical_landed_height(self):
        truth = rigid((0.0, 0.0, 0.045))
        estimated = rigid((0.0, 0.0, -0.005))

        correction = localization.map_to_odom(truth, estimated, np.eye(4))

        np.testing.assert_allclose(correction @ estimated, truth, atol=1e-12)
        self.assertAlmostEqual(0.045, (correction @ estimated)[2, 3])
        self.assertAlmostEqual(0.05, correction[2, 3])

    def test_recomposes_translated_rotated_frames_under_changing_drift(self):
        map_world = rigid((3.0, -4.0, 1.25), (0.2, -0.5, 0.7))
        for index in range(8):
            with self.subTest(index=index):
                truth = rigid((1.0 + index, -0.4 * index, 0.045 + index),
                              (0.1 * index, -0.15 * index, 0.3 * index))
                estimated = rigid((-2.0 + 0.9 * index, 0.2 * index, -0.005 + index),
                                  (-0.08 * index, 0.1 * index, -0.2 * index))

                correction = localization.map_to_odom(truth, estimated, map_world)

                np.testing.assert_allclose(correction @ estimated,
                                           map_world @ truth, atol=1e-12)
                np.testing.assert_allclose(correction[:3, :3].T @ correction[:3, :3],
                                           np.eye(3), atol=1e-12)
                self.assertAlmostEqual(1.0, np.linalg.det(correction[:3, :3]))

    def test_real_ground_endpoint_stays_on_map_ground_through_sensor_chain(self):
        map_world = rigid((2.0, -3.0, 0.0), (0.0, 0.0, 0.8))
        base_sensor = rigid((0.12, -0.02, -0.015), (0.2, -0.1, 0.4))
        ground_world = np.array([1.2, -0.7, 0.0, 1.0])
        for height in (0.045, 1.5):
            with self.subTest(height=height):
                truth = rigid((0.2, -0.1, height), (0.1, -0.2, 0.6))
                estimated = rigid((0.9, 0.4, height - 0.05), (-0.2, 0.3, 0.2))
                measured_sensor = np.linalg.inv(truth @ base_sensor) @ ground_world

                correction = localization.map_to_odom(truth, estimated, map_world)
                endpoint = correction @ estimated @ base_sensor @ measured_sensor

                np.testing.assert_allclose(endpoint, map_world @ ground_world, atol=1e-12)
                self.assertAlmostEqual(0.0, endpoint[2])

    def test_rejects_nonfinite_or_nonrigid_inputs_in_each_position(self):
        invalid = [np.eye(3), np.ones((4, 4))]
        for row, column, value in ((0, 3, np.nan), (2, 0, np.inf),
                                   (3, 0, 0.01), (0, 0, 2.0), (0, 0, -1.0)):
            matrix = np.eye(4)
            matrix[row, column] = value
            invalid.append(matrix)
        for slot in range(3):
            for matrix in invalid:
                with self.subTest(slot=slot, matrix=matrix):
                    inputs = [np.eye(4), np.eye(4), np.eye(4)]
                    inputs[slot] = matrix
                    with self.assertRaises(ValueError):
                        localization.map_to_odom(*inputs)

    def test_inputs_are_not_modified(self):
        inputs = [rigid((index, 2.0, 3.0), (0.2, index * 0.3, -0.1))
                  for index in range(3)]
        originals = [matrix.copy() for matrix in inputs]
        localization.map_to_odom(*inputs)
        for matrix, original in zip(inputs, originals):
            np.testing.assert_array_equal(matrix, original)


class PoseConversionTest(unittest.TestCase):
    def test_xyzw_quaternion_convention(self):
        matrix = localization.pose_matrix((1.0, 2.0, 3.0),
                                          (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)))
        np.testing.assert_allclose(matrix, rigid((1.0, 2.0, 3.0),
                                                (0.0, 0.0, math.pi / 2)), atol=1e-12)

    def test_quaternion_round_trip_including_half_turns(self):
        for angles in ((0, 0, 0), (math.pi, 0, 0), (0, math.pi, 0),
                       (0, 0, math.pi), (0.3, -0.7, 1.5)):
            with self.subTest(angles=angles):
                matrix = rigid((1.0, -2.0, 0.045), angles)
                quaternion = localization.rotation_quaternion(matrix)
                self.assertAlmostEqual(1.0, np.linalg.norm(quaternion))
                np.testing.assert_allclose(localization.pose_matrix(matrix[:3, 3], quaternion),
                                           matrix, atol=1e-12)

    def test_rejects_invalid_pose_coordinates_and_quaternions(self):
        cases = [((0, 0, 0), (0, 0, 0, 0)), ((0, 0, 0), (0, 0, 0, 2)),
                 ((0, 0, 0), (0, 0, np.nan, 1)), ((0, np.inf, 0), (0, 0, 0, 1)),
                 ((0, 0), (0, 0, 0, 1)), ((0, 0, 0), (0, 0, 1))]
        for translation, quaternion in cases:
            with self.subTest(translation=translation, quaternion=quaternion):
                with self.assertRaises(ValueError):
                    localization.pose_matrix(translation, quaternion)


class StampSelectionTest(unittest.TestCase):
    def test_accepts_only_positive_new_nonfuture_fresh_nanosecond_stamp(self):
        now = 10000000000
        cases = [(now, now - 1, True), (now - 500000000, 0, True),
                 (now - 500000001, 0, False), (now + 1, 0, False),
                 (0, 0, False), (-1, 0, False), (now, now, False),
                 (now - 1, now, False)]
        for stamp, previous, expected in cases:
            with self.subTest(stamp=stamp, previous=previous):
                self.assertEqual(expected, localization.fresh_stamp(stamp, now, previous))


@unittest.skipIf(tf2_ros is None, "source Noetic for real in-memory tf2 interpolation checks")
class SameTimeBufferTest(unittest.TestCase):
    def setUp(self):
        # debug=False prevents ROS-master/service access; these are local buffers.
        self.public = tf2_ros.Buffer(debug=False)
        self.private = tf2_ros.Buffer(debug=False)
        self.map_world = rigid((2.0, -1.0, 0.0), (0.0, 0.0, 0.4))
        self.insert(self.public, "map", "world", 0.0, self.map_world, static=True)
        self.estimated = rigid((0.0, 0.0, -0.005))
        self.insert(self.public, "uav1/odom", "uav1/base_link", 10.1, self.estimated)

    @staticmethod
    def insert(buffer, parent, child, stamp, matrix, static=False):
        message = TransformStamped()
        message.header.frame_id = parent
        message.child_frame_id = child
        message.header.stamp = stamp if isinstance(stamp, rospy.Time) else rospy.Time.from_sec(stamp)
        message.transform.translation.x, message.transform.translation.y, message.transform.translation.z = matrix[:3, 3]
        quaternion = localization.rotation_quaternion(matrix)
        message.transform.rotation.x, message.transform.rotation.y, message.transform.rotation.z, message.transform.rotation.w = quaternion
        if static:
            buffer.set_transform_static(message, "offline_test")
        else:
            buffer.set_transform(message, "offline_test")

    def test_uses_interpolated_truth_at_exact_odom_stamp(self):
        self.insert(self.private, "world", "sim_uav1_body", 10.0,
                    rigid((0, 0, 0.045), (0, 0, 0)))
        self.insert(self.private, "world", "sim_uav1_body", 10.2,
                    rigid((2, 0, 2.045), (0, 0, 0.4)))

        stamp, correction = localization.localization_at_latest_common_time(
            self.public, self.private, rospy.Time.from_sec(10.25), 0)

        self.assertEqual(rospy.Time.from_sec(10.1), stamp)
        expected_truth = rigid((1.0, 0.0, 1.045), (0, 0, 0.2))
        np.testing.assert_allclose(correction @ self.estimated,
                                   self.map_world @ expected_truth, atol=1e-9)
        self.assertNotIn("sim_uav1_body", self.public.all_frames_as_yaml())

    def test_missing_overlap_waits_for_next_attempt_without_fallback(self):
        self.insert(self.private, "world", "sim_uav1_body", 10.0, rigid((0, 0, 0.045)))
        with self.assertRaises(tf2_ros.ExtrapolationException):
            localization.localization_at_latest_common_time(
                self.public, self.private, rospy.Time.from_sec(10.2), 0)
        self.insert(self.private, "world", "sim_uav1_body", 10.2, rigid((0, 0, 0.045)))
        result = localization.localization_at_latest_common_time(
            self.public, self.private, rospy.Time.from_sec(10.25), 0)
        self.assertEqual(rospy.Time.from_sec(10.1), result[0])

    def test_truth_history_newer_than_odom_has_no_unequal_time_fallback(self):
        self.insert(self.private, "world", "sim_uav1_body", 10.2,
                    rigid((0, 0, 0.045)))
        with self.assertRaises(tf2_ros.ExtrapolationException):
            localization.localization_at_latest_common_time(
                self.public, self.private, rospy.Time.from_sec(10.25), 0)

    def test_advancing_streams_keep_publishing_with_delayed_truth(self):
        # Odometry advances at 20 Hz and truth at 100 Hz, but truth messages
        # arrive 60 ms late. A 50 Hz timer must use the available common history.
        self.public.clear()  # tf2 keeps the static map/world edge.
        start_ns = 10000000000
        next_odom_ns = next_truth_ns = start_ns
        previous_ns = 0
        corrections = []

        def physical_at(stamp_ns):
            elapsed = (stamp_ns - start_ns) / 1e9
            return rigid((0.5 * elapsed, -0.25 * elapsed, 0.045 + 0.1 * elapsed),
                         (0.0, 0.0, 0.2 * elapsed))

        def estimated_at(stamp_ns):
            elapsed = (stamp_ns - start_ns) / 1e9
            return rigid((-0.3 + 0.6 * elapsed, 0.2 - 0.15 * elapsed,
                          -0.005 + 0.12 * elapsed), (0.0, 0.0, -0.1 + 0.3 * elapsed))

        for attempt in range(100):
            now_ns = start_ns + attempt * 20000000
            while next_odom_ns <= now_ns:
                self.insert(self.public, "uav1/odom", "uav1/base_link",
                            rospy.Time(0, next_odom_ns), estimated_at(next_odom_ns))
                next_odom_ns += 50000000
            while next_truth_ns <= now_ns - 60000000:
                self.insert(self.private, "world", "sim_uav1_body",
                            rospy.Time(0, next_truth_ns), physical_at(next_truth_ns))
                next_truth_ns += 10000000
            try:
                sample = localization.localization_at_latest_common_time(
                    self.public, self.private, rospy.Time(0, now_ns), previous_ns)
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
                continue
            if sample is None:
                continue
            stamp, correction = sample
            stamp_ns = stamp.to_nsec()
            self.assertEqual(min(next_odom_ns - 50000000, next_truth_ns - 10000000),
                             stamp_ns)
            self.assertGreater(stamp_ns, previous_ns)
            self.assertLessEqual(now_ns - stamp_ns, 500000000)
            np.testing.assert_allclose(correction @ estimated_at(stamp_ns),
                                       self.map_world @ physical_at(stamp_ns), atol=1e-10)
            corrections.append(stamp_ns)
            previous_ns = stamp_ns

        self.assertEqual(97, len(corrections),
                         "bounded truth delivery delay must not starve localization")

    def test_stale_or_repeated_common_stamp_is_skipped(self):
        self.insert(self.private, "world", "sim_uav1_body", 10.05,
                    rigid((0, 0, 0.045)))
        self.assertIsNone(localization.localization_at_latest_common_time(
            self.public, self.private, rospy.Time.from_sec(10.7), 0))
        self.assertIsNone(localization.localization_at_latest_common_time(
            self.public, self.private, rospy.Time.from_sec(10.2),
            rospy.Time.from_sec(10.05).to_nsec()))

    def test_fresh_odom_does_not_make_stale_truth_usable(self):
        self.insert(self.public, "uav1/odom", "uav1/base_link", 10.6, self.estimated)
        self.insert(self.private, "world", "sim_uav1_body", 10.1,
                    rigid((0, 0, 0.045)))
        self.assertIsNone(localization.localization_at_latest_common_time(
            self.public, self.private, rospy.Time.from_sec(10.65), 0))

    def test_unavailable_map_world_has_no_identity_fallback(self):
        self.public = tf2_ros.Buffer(debug=False)
        self.insert(self.public, "uav1/odom", "uav1/base_link", 10.1, self.estimated)
        self.insert(self.private, "world", "sim_uav1_body", 10.0, rigid((0, 0, 0.045)))
        self.insert(self.private, "world", "sim_uav1_body", 10.2, rigid((0, 0, 0.045)))
        with self.assertRaises(tf2_ros.LookupException):
            localization.localization_at_latest_common_time(
                self.public, self.private, rospy.Time.from_sec(10.25), 0)

    def offline_node(self):
        # Isolate only the ROS network, clock and logging boundaries. Geometry,
        # ROS messages, both tf2 buffers and the callbacks remain real.
        node = object.__new__(localization.SimUavLocalization)
        node.public_buffer = self.public
        node.truth_buffer = self.private
        node.transform_type = TransformStamped
        node.previous_ns = 0
        node.lookup_errors = (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                              tf2_ros.ExtrapolationException, ValueError)
        node.ros = SimpleNamespace(
            Time=SimpleNamespace(now=lambda: rospy.Time.from_sec(10.25)),
            logwarn_throttle=lambda *args: None,
            logdebug_throttle=lambda *args: None)
        return node

    def test_truth_callback_validates_frame_stamp_and_pose_without_public_tf(self):
        node = self.offline_node()
        message = Odometry()
        message.header.frame_id = "world"
        message.child_frame_id = "base_link"
        message.header.stamp = rospy.Time(10, 123456789)
        message.pose.pose.position.z = 0.045
        message.pose.pose.orientation.w = 1.0
        for parent, child, stamp, q_w in (("map", "base_link", rospy.Time(10), 1),
                                         ("world", "uav1/base_link", rospy.Time(10), 1),
                                         ("world", "base_link", rospy.Time(0), 1),
                                         ("world", "base_link", rospy.Time(10), 0)):
            with self.subTest(parent=parent, child=child, stamp=stamp, q_w=q_w):
                message.header.frame_id, message.child_frame_id = parent, child
                message.header.stamp, message.pose.pose.orientation.w = stamp, q_w
                node.receive_truth(message)
                self.assertFalse(self.private.can_transform(
                    "world", "sim_uav1_body", rospy.Time()))
        message.header.stamp = rospy.Time(10, 123456789)
        message.pose.pose.orientation.w = 1.0
        node.receive_truth(message)
        cached = self.private.lookup_transform("world", "sim_uav1_body", rospy.Time())
        self.assertEqual(message.header.stamp, cached.header.stamp)
        self.assertAlmostEqual(0.045, cached.transform.translation.z)
        self.assertNotIn("sim_uav1_body", self.public.all_frames_as_yaml())

    def test_timer_broadcasts_only_map_odom_once_after_matching_truth_arrives(self):
        node = self.offline_node()
        published = []
        node.broadcaster = SimpleNamespace(sendTransform=published.append)
        node.publish_correction(None)
        self.assertEqual([], published)
        self.assertEqual(0, node.previous_ns)
        self.insert(self.private, "world", "sim_uav1_body", 10.0, rigid((0, 0, 0.045)))
        self.insert(self.private, "world", "sim_uav1_body", 10.2, rigid((0, 0, 0.045)))
        node.publish_correction(None)
        node.publish_correction(None)
        self.assertEqual(1, len(published))
        correction = published[0]
        self.assertEqual("map", correction.header.frame_id)
        self.assertEqual("uav1/odom", correction.child_frame_id)
        self.assertEqual(rospy.Time.from_sec(10.1), correction.header.stamp)
        self.assertEqual(correction.header.stamp.to_nsec(), node.previous_ns)
        np.testing.assert_allclose(localization.transform_matrix(correction) @ self.estimated,
                                   self.map_world @ rigid((0, 0, 0.045)), atol=1e-12)
        correction.serialize(io.BytesIO())


if __name__ == "__main__":
    unittest.main()
