#!/usr/bin/env python3
"""Publish a stable red target pose from one configured RGB-D camera."""

from collections import deque
import json
import math
import threading

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped, PoseStamped
import message_filters
import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from tf.transformations import quaternion_from_euler, quaternion_matrix
import tf2_ros

from air_ground_pick_demo.perception import (
    PerceptionError, backproject_mask, estimate_target_pose,
    fuse_pose_samples, register_depth_to_color, select_red_component,
    validate_observation_stamps)


class RedTargetObserver:
    def __init__(self):
        runtime_ready_topic = rospy.get_param(
            "~runtime_ready_topic", "/ground/runtime_ready")
        runtime_ready_timeout = float(rospy.get_param(
            "~runtime_ready_timeout", 90.0))
        if not math.isfinite(runtime_ready_timeout) or runtime_ready_timeout <= 0:
            raise ValueError("runtime_ready_timeout must be positive")
        ready = rospy.wait_for_message(
            runtime_ready_topic, Bool, timeout=runtime_ready_timeout)
        if not ready.data:
            raise RuntimeError(
                "ground runtime did not become ready on %s" %
                runtime_ready_topic)

        self.color_topic = rospy.get_param("~color_topic")
        self.depth_topic = rospy.get_param("~depth_topic")
        self.color_info_topic = rospy.get_param("~color_info_topic")
        self.depth_info_topic = rospy.get_param("~depth_info_topic")
        self.output_topic = rospy.get_param("~output_topic")
        self.status_topic = rospy.get_param(
            "~status_topic", self.output_topic + "_status")
        self.target_frame = rospy.get_param("~target_frame", "world").lstrip("/")
        self.camera_optical_frame = rospy.get_param(
            "~camera_optical_frame").lstrip("/")
        self.target_height = float(rospy.get_param("~target_height", 0.115))
        self.top_surface_tolerance = float(rospy.get_param(
            "~top_surface_tolerance", 0.015))
        self.target_top_size = rospy.get_param("~target_top_size", None)
        self.minimum_top_span_fraction = float(rospy.get_param(
            "~minimum_top_span_fraction", 0.90))
        self.maximum_top_tilt_degrees = float(rospy.get_param(
            "~maximum_top_tilt_degrees", 15.0))
        surface_cue_topic = rospy.get_param("~surface_cue_topic", "")
        if ((self.target_top_size is not None or surface_cue_topic) and
                self.target_frame != "map"):
            raise ValueError("Ground top validation and surface cue require map frame")
        self.min_depth = float(rospy.get_param("~min_depth", 0.15))
        self.max_depth = float(rospy.get_param("~max_depth", 4.0))
        self.min_pixels = int(rospy.get_param("~min_pixels", 40))
        self.minimum_points = int(rospy.get_param("~minimum_points", 30))
        self.ambiguity_ratio = float(rospy.get_param(
            "~ambiguity_ratio", 0.65))
        self.stable_frames = int(rospy.get_param("~stable_frames", 3))
        self.max_position_spread = float(rospy.get_param(
            "~max_position_spread", 0.10))
        self.max_yaw_spread = float(rospy.get_param(
            "~max_yaw_spread", 0.30))
        self.tf_timeout = float(rospy.get_param("~tf_timeout", 0.20))
        self.max_observation_age = float(rospy.get_param(
            "~max_observation_age", 1.0))
        self.max_future_skew = float(rospy.get_param(
            "~max_future_skew", 0.1))
        self.samples = deque(maxlen=self.stable_frames)
        self.samples_lock = threading.Lock()
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(15.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pose_publisher = rospy.Publisher(
            self.output_topic, PoseStamped, queue_size=1)
        self.surface_cue_publisher = (
            rospy.Publisher(surface_cue_topic, PointStamped, queue_size=1)
            if surface_cue_topic else None)
        self.status_publisher = rospy.Publisher(
            self.status_topic, String, queue_size=1, latch=True)

        color = message_filters.Subscriber(self.color_topic, Image)
        depth = message_filters.Subscriber(self.depth_topic, Image)
        color_info = message_filters.Subscriber(
            self.color_info_topic, CameraInfo)
        depth_info = message_filters.Subscriber(
            self.depth_info_topic, CameraInfo)
        self.subscribers = (color, depth, color_info, depth_info)
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            self.subscribers,
            queue_size=int(rospy.get_param("~sync_queue_size", 10)),
            slop=float(rospy.get_param("~sync_slop", 0.05)))
        self.synchronizer.registerCallback(self.observe)
        self.publish_status("WAITING")

    @staticmethod
    def transform_parts(transform):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        quaternion = (rotation.x, rotation.y, rotation.z, rotation.w)
        if not all(math.isfinite(float(value)) for value in quaternion):
            raise PerceptionError("TF quaternion is non-finite")
        matrix = quaternion_matrix(quaternion)
        return matrix[:3, :3], np.array((
            translation.x, translation.y, translation.z), dtype=np.float64)

    def lookup_transform(self, target, source, stamp):
        return self.tf_buffer.lookup_transform(
            target, source, stamp, rospy.Duration(self.tf_timeout))

    def validate_stream_metadata(self, color, depth, color_info, depth_info):
        color_frame = color.header.frame_id.lstrip("/")
        depth_frame = depth.header.frame_id.lstrip("/")
        color_info_frame = color_info.header.frame_id.lstrip("/")
        depth_info_frame = depth_info.header.frame_id.lstrip("/")
        if not color_frame or not depth_frame:
            raise PerceptionError("RGB-D image frame is empty")
        if color_frame != self.camera_optical_frame:
            raise PerceptionError(
                "color frame %s does not match configured frame %s" %
                (color_frame, self.camera_optical_frame))
        if color_info_frame != color_frame:
            raise PerceptionError("color image and CameraInfo frames differ")
        if depth_info_frame != depth_frame:
            raise PerceptionError("depth image and CameraInfo frames differ")
        stamps = (
            color.header.stamp.to_sec(), depth.header.stamp.to_sec(),
            color_info.header.stamp.to_sec(), depth_info.header.stamp.to_sec())
        validate_observation_stamps(
            stamps, rospy.Time.now().to_sec(), self.max_observation_age,
            self.max_future_skew)
        return color_frame, depth_frame, stamps

    @staticmethod
    def validate_image_shapes(color, depth, color_info, depth_info,
                              rgb, depth_image):
        color_shape = (int(color.height), int(color.width))
        depth_shape = (int(depth.height), int(depth.width))
        if rgb.shape[:2] != color_shape:
            raise PerceptionError("decoded color image dimensions differ")
        if depth_image.shape != depth_shape:
            raise PerceptionError("decoded depth image dimensions differ")
        if color_shape != (int(color_info.height), int(color_info.width)):
            raise PerceptionError("color image and CameraInfo dimensions differ")
        if depth_shape != (int(depth_info.height), int(depth_info.width)):
            raise PerceptionError("depth image and CameraInfo dimensions differ")

    @staticmethod
    def depth_metres(message, bridge):
        depth = bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
        if message.encoding in ("16UC1", "mono16"):
            return np.asarray(depth, dtype=np.float32) * 0.001
        if message.encoding == "32FC1":
            return np.asarray(depth, dtype=np.float32)
        raise PerceptionError(
            "unsupported depth encoding %s" % message.encoding)

    def publish_status(self, state, **values):
        if rospy.is_shutdown():
            return
        payload = {
            "state": state,
            "stamp": rospy.Time.now().to_sec(),
            "target_frame": self.target_frame,
        }
        payload.update(values)
        try:
            self.status_publisher.publish(String(data=json.dumps(
                payload, sort_keys=True, allow_nan=False)))
        except rospy.ROSException:
            if not rospy.is_shutdown():
                raise

    def publish_pose(self, pose):
        if rospy.is_shutdown():
            return
        try:
            self.pose_publisher.publish(pose)
        except rospy.ROSException:
            if not rospy.is_shutdown():
                raise

    def publish_surface_cue(self, points, stamp):
        """Publish measured surface support for aiming, never a cuboid pose."""
        if self.surface_cue_publisher is None or rospy.is_shutdown():
            return
        if self.target_frame != "map":
            raise PerceptionError("surface cue requires map frame")
        cloud = np.asarray(points, dtype=np.float64)
        if cloud.ndim != 2 or cloud.shape[1:] != (3,):
            raise PerceptionError("surface cue points are invalid")
        cloud = cloud[np.isfinite(cloud).all(axis=1)]
        if not len(cloud):
            raise PerceptionError("surface cue has no finite measured points")
        cue = PointStamped()
        cue.header.frame_id = self.target_frame
        cue.header.stamp = stamp
        cue.point.x, cue.point.y, cue.point.z = (
            float(value) for value in np.median(cloud, axis=0))
        try:
            self.surface_cue_publisher.publish(cue)
        except rospy.ROSException:
            if not rospy.is_shutdown():
                raise

    def observe(self, color, depth, color_info, depth_info):
        try:
            color_frame, depth_frame, stamps = self.validate_stream_metadata(
                color, depth, color_info, depth_info)
            rgb = self.bridge.imgmsg_to_cv2(
                color, desired_encoding="rgb8")
            depth_image = self.depth_metres(depth, self.bridge)
            self.validate_image_shapes(
                color, depth, color_info, depth_info, rgb, depth_image)
            mask = select_red_component(
                rgb, min_pixels=self.min_pixels,
                ambiguity_ratio=self.ambiguity_ratio)

            depth_to_color = self.lookup_transform(
                color_frame, depth_frame, depth.header.stamp)
            depth_rotation, depth_translation = self.transform_parts(
                depth_to_color)
            registered_depth = register_depth_to_color(
                depth_image, depth_info.K, color_info.K, rgb.shape[:2],
                depth_rotation, depth_translation)
            color_points = backproject_mask(
                mask, registered_depth, color_info.K,
                self.min_depth, self.max_depth)
            if len(color_points) < self.minimum_points:
                raise PerceptionError(
                    "target has only %d depth points" % len(color_points))

            target_from_color = self.lookup_transform(
                self.target_frame, color_frame, color.header.stamp)
            target_rotation, target_translation = self.transform_parts(
                target_from_color)
            target_points = color_points.dot(
                target_rotation.T) + target_translation
            if self.surface_cue_publisher is not None:
                validate_observation_stamps(
                    stamps, rospy.Time.now().to_sec(), self.max_observation_age,
                    self.max_future_skew)
                self.publish_surface_cue(target_points, color.header.stamp)
            if self.target_top_size is not None and (
                    np.any(mask[0, :]) or np.any(mask[-1, :]) or
                    np.any(mask[:, 0]) or np.any(mask[:, -1])):
                raise PerceptionError("target top observation is image-clipped")
            estimate = estimate_target_pose(
                target_points, self.target_height,
                self.top_surface_tolerance,
                target_top_size=self.target_top_size,
                minimum_top_span_fraction=self.minimum_top_span_fraction,
                maximum_top_tilt_degrees=self.maximum_top_tilt_degrees)
            with self.samples_lock:
                self.samples.append(estimate)
                if len(self.samples) < self.stable_frames:
                    self.publish_status(
                        "ACCUMULATING", frames=len(self.samples),
                        mask_pixels=int(np.count_nonzero(mask)),
                        points=len(color_points))
                    return
                fused = fuse_pose_samples(
                    tuple(self.samples), self.max_position_spread,
                    self.max_yaw_spread)

            pose = PoseStamped()
            pose.header.stamp = color.header.stamp
            pose.header.frame_id = self.target_frame
            pose.pose.position.x = float(fused[0])
            pose.pose.position.y = float(fused[1])
            pose.pose.position.z = float(fused[2])
            quaternion = quaternion_from_euler(0.0, 0.0, float(fused[3]))
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]
            validate_observation_stamps(
                stamps, rospy.Time.now().to_sec(), self.max_observation_age,
                self.max_future_skew)
            self.publish_pose(pose)
            self.publish_status(
                "TRACKING", frames=len(self.samples),
                mask_pixels=int(np.count_nonzero(mask)),
                points=len(color_points), observation_stamp=pose.header.stamp.to_sec())
        except (CvBridgeError, PerceptionError, tf2_ros.TransformException) as error:
            with self.samples_lock:
                self.samples.clear()
            self.publish_status("REJECTED", reason=str(error))
            rospy.logwarn_throttle(2.0, "red target observation rejected: %s", error)


if __name__ == "__main__":
    rospy.init_node("red_target_observer")
    RedTargetObserver()
    rospy.spin()
