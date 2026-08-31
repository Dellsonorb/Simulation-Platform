#!/usr/bin/env python3
"""Synchronized P450 RGB-D brick extraction and world-frame pose estimation."""

import copy
import json
import math
import threading
from collections import deque

import cv2
import message_filters
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as point_cloud2
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped, Vector3
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Header, String
from tf.transformations import quaternion_matrix, quaternion_from_euler
from visualization_msgs.msg import Marker

from brick_aerial_perception.geometry import (
    backproject_mask,
    estimate_brick_pose,
    register_depth_to_color,
    yaw_error_mod_pi,
)
from brick_aerial_perception.brick_contract import oriented_brick_contract
from brick_aerial_perception.mask_frontend import (
    make_mask_frontend,
    projected_target_aspect_ratio,
)


class AerialBrickPoseNode:
    def __init__(self):
        self.uav_id = int(rospy.get_param("~uav_id", 1))
        prefix = "/uav{}/".format(self.uav_id)
        self.color_topic = rospy.get_param(
            "~color_topic", prefix + "camera/color/image_raw"
        )
        self.depth_topic = rospy.get_param(
            "~depth_topic", prefix + "camera/depth/image_raw"
        )
        self.color_info_topic = rospy.get_param(
            "~color_info_topic", prefix + "camera/color/camera_info"
        )
        self.depth_info_topic = rospy.get_param(
            "~depth_info_topic", prefix + "camera/depth/camera_info"
        )
        self.world_frame = rospy.get_param("~world_frame", "world").lstrip("/")
        self.optical_frame = rospy.get_param(
            "~camera_optical_frame",
            "uav{}/camera_color_optical_frame".format(self.uav_id),
        ).lstrip("/")
        self.min_depth = float(rospy.get_param("~min_depth", 0.25))
        self.max_depth = float(rospy.get_param("~max_depth", 8.0))
        self.min_mask_pixels = int(rospy.get_param("~min_mask_pixels", 40))
        self.min_points = int(rospy.get_param("~min_points", 30))
        self.stable_frames = int(rospy.get_param("~stable_frames", 5))
        self.max_position_spread = float(rospy.get_param("~max_position_spread", 0.10))
        self.max_yaw_spread = float(rospy.get_param("~max_yaw_spread", 0.20))
        self.brick_contract = oriented_brick_contract(
            rospy.get_param("~brick_length", 0.240),
            rospy.get_param("~brick_width", 0.115),
            rospy.get_param("~brick_height", 0.053),
            rospy.get_param("~brick_orientation_mode", "flat"),
        )
        self.require_sampling_state = bool(rospy.get_param("~require_sampling_state", True))
        self.mask_frontend_name = str(rospy.get_param(
            "~mask_frontend", "rgb_shape_component"))
        projected_aspect_target = projected_target_aspect_ratio(
            self.brick_contract.orientation_mode,
            rospy.get_param("~target_aspect_ratio", 4.53),
            rospy.get_param("~side_up_target_aspect_ratio", 2.0),
        )
        frontend_config = {
            "morph_kernel": int(rospy.get_param("~morph_kernel", 3)),
            "red_hue_low_1": int(rospy.get_param("~red_hue_low_1", 0)),
            "red_hue_high_1": int(rospy.get_param("~red_hue_high_1", 12)),
            "red_hue_low_2": int(rospy.get_param("~red_hue_low_2", 165)),
            "red_hue_high_2": int(rospy.get_param("~red_hue_high_2", 179)),
            "min_saturation": int(rospy.get_param("~min_saturation", 80)),
            "min_value": int(rospy.get_param("~min_value", 55)),
            "min_component_area": float(rospy.get_param(
                "~min_component_area", 30.0)),
            "max_component_area": float(rospy.get_param(
                "~max_component_area", 12000.0)),
            "min_aspect_ratio": float(rospy.get_param(
                "~min_aspect_ratio", 1.8)),
            "max_aspect_ratio": float(rospy.get_param(
                "~max_aspect_ratio", 8.5)),
            "target_aspect_ratio": projected_aspect_target,
            "min_rectangularity": float(rospy.get_param(
                "~min_rectangularity", 0.55)),
            "boundary_margin": int(rospy.get_param("~boundary_margin", 2)),
            "min_score": float(rospy.get_param("~min_frontend_score", 0.60)),
            "ambiguity_margin": float(rospy.get_param(
                "~frontend_ambiguity_margin", 0.06)),
        }
        self.mask_frontend = make_mask_frontend(
            self.mask_frontend_name, frontend_config)

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.lock = threading.RLock()
        self.samples = deque(maxlen=max(1, self.stable_frames))
        self.sampling_enabled = not self.require_sampling_state
        self.viewpoint_id = "manual"

        self.aligned_depth_pub = rospy.Publisher(
            "~aligned_depth_to_color/image_raw", Image, queue_size=1
        )
        self.aligned_info_pub = rospy.Publisher(
            "~aligned_depth_to_color/camera_info", CameraInfo, queue_size=1
        )
        self.mask_pub = rospy.Publisher("~brick_mask", Image, queue_size=1)
        self.debug_pub = rospy.Publisher("~brick_debug_rgb", Image, queue_size=1)
        self.cloud_pub = rospy.Publisher("~brick_points", PointCloud2, queue_size=1)
        self.pose_pub = rospy.Publisher("~brick_pose", PoseStamped, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher("~quality_status", String, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher("~brick_pose_marker", Marker, queue_size=1)
        rospy.Subscriber("/m1/viewpoint_state", String, self._viewpoint_state_cb, queue_size=5)

        subscribers = [
            message_filters.Subscriber(self.color_topic, Image),
            message_filters.Subscriber(self.depth_topic, Image),
            message_filters.Subscriber(self.color_info_topic, CameraInfo),
            message_filters.Subscriber(self.depth_info_topic, CameraInfo),
        ]
        synchronizer = message_filters.ApproximateTimeSynchronizer(
            subscribers,
            int(rospy.get_param("~sync_queue_size", 10)),
            float(rospy.get_param("~sync_slop", 0.04)),
        )
        synchronizer.registerCallback(self._rgbd_cb)
        self.subscribers = subscribers
        self.synchronizer = synchronizer
        self._publish_status("WAITING_FOR_VIEWPOINT")

    def _viewpoint_state_cb(self, message):
        with self.lock:
            enabled = message.data.startswith("SAMPLING:")
            viewpoint = message.data.split(":", 1)[1] if enabled else ""
            if enabled != self.sampling_enabled or viewpoint != self.viewpoint_id:
                self.samples.clear()
            self.sampling_enabled = enabled or not self.require_sampling_state
            self.viewpoint_id = viewpoint or "manual"
            if not self.sampling_enabled:
                self._publish_status("WAITING_FOR_VIEWPOINT")

    def _publish_status(self, state, **details):
        payload = {
            "state": state,
            "viewpoint": self.viewpoint_id,
            "brick_orientation_mode": self.brick_contract.orientation_mode,
            "brick_nominal_dimensions": list(
                self.brick_contract.nominal_dimensions
            ),
            "brick_top_dimensions": list(self.brick_contract.top_dimensions),
            "brick_vertical_height": self.brick_contract.vertical_height,
            "brick_grasp_span": self.brick_contract.grasp_span,
            "mask_frontend": self.mask_frontend_name,
        }
        payload.update(details)
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    @staticmethod
    def _depth_metres(message, bridge):
        image = bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
        if message.encoding in ("16UC1", "mono16"):
            return image.astype(np.float32) * 0.001
        if message.encoding == "32FC1":
            return image.astype(np.float32)
        raise CvBridgeError("unsupported depth encoding {}".format(message.encoding))

    def _to_world(self, points, stamp):
        transform = self.tf_buffer.lookup_transform(
            self.world_frame, self.optical_frame, stamp, rospy.Duration(0.10)
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        matrix = quaternion_matrix((rotation.x, rotation.y, rotation.z, rotation.w))[:3, :3]
        offset = np.array([translation.x, translation.y, translation.z], dtype=np.float64)
        return points.dot(matrix.T) + offset

    def _stable_pose(self):
        if len(self.samples) < self.samples.maxlen:
            return None, None, None
        values = np.asarray(self.samples, dtype=np.float64)
        position = np.median(values[:, :3], axis=0)
        position_spread = float(np.max(np.linalg.norm(values[:, :3] - position, axis=1)))
        doubled = 2.0 * values[:, 3]
        yaw = 0.5 * math.atan2(float(np.mean(np.sin(doubled))),
                               float(np.mean(np.cos(doubled))))
        yaw_spread = max(yaw_error_mod_pi(item, yaw) for item in values[:, 3])
        if position_spread > self.max_position_spread or yaw_spread > self.max_yaw_spread:
            return None, position_spread, yaw_spread
        return np.array([position[0], position[1], position[2], yaw]), position_spread, yaw_spread

    def _publish_debug(self, color_message, rgb, mask, aligned_depth, color_info, points_world):
        header = copy.deepcopy(color_message.header)
        header.frame_id = self.optical_frame
        aligned_message = self.bridge.cv2_to_imgmsg(aligned_depth, encoding="32FC1")
        aligned_message.header = header
        self.aligned_depth_pub.publish(aligned_message)
        aligned_info = copy.deepcopy(color_info)
        aligned_info.header = header
        self.aligned_info_pub.publish(aligned_info)
        mask_message = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
        mask_message.header = header
        self.mask_pub.publish(mask_message)
        debug = rgb.copy()
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        cv2.drawContours(debug, contours, -1, (0, 255, 0), 2)
        debug_message = self.bridge.cv2_to_imgmsg(debug, encoding="rgb8")
        debug_message.header = header
        self.debug_pub.publish(debug_message)
        cloud_header = Header(stamp=color_message.header.stamp, frame_id=self.world_frame)
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(cloud_header, points_world))

    def _publish_pose_and_marker(self, pose, source_header):
        quaternion = quaternion_from_euler(0.0, 0.0, float(pose[3]))
        message = PoseStamped()
        message.header.seq = int(source_header.seq)
        message.header.stamp = source_header.stamp
        message.header.frame_id = self.world_frame
        message.pose.position.x = float(pose[0])
        message.pose.position.y = float(pose[1])
        message.pose.position.z = float(pose[2])
        message.pose.orientation.x = quaternion[0]
        message.pose.orientation.y = quaternion[1]
        message.pose.orientation.z = quaternion[2]
        message.pose.orientation.w = quaternion[3]
        self.pose_pub.publish(message)
        published_seq = int(message.header.seq)
        marker = Marker()
        marker.header = copy.deepcopy(message.header)
        marker.ns = "m1_brick_pose"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = message.pose
        marker.scale = Vector3(0.25, 0.035, 0.035)
        marker.color.r = 0.1
        marker.color.g = 1.0
        marker.color.b = 0.1
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(0.5)
        self.marker_pub.publish(marker)
        return published_seq

    def _rgbd_cb(self, color_message, depth_message, color_info, depth_info):
        with self.lock:
            if rospy.is_shutdown():
                return
            try:
                rgb = self.bridge.imgmsg_to_cv2(color_message, desired_encoding="rgb8")
                depth = self._depth_metres(depth_message, self.bridge)
                color_k = np.asarray(color_info.K, dtype=np.float64).reshape(3, 3)
                depth_k = np.asarray(depth_info.K, dtype=np.float64).reshape(3, 3)
                aligned_depth = register_depth_to_color(
                    depth, depth_k, color_k, rgb.shape[:2]
                )
                mask_result = self.mask_frontend.segment(rgb)
                mask = mask_result.mask
                frontend_details = {
                    "mask_frontend": self.mask_frontend_name,
                    "frontend": mask_result.audit,
                }
                if mask_result.state != "VALID":
                    self.samples.clear()
                    self._publish_debug(
                        color_message, rgb, mask, aligned_depth, color_info,
                        np.empty((0, 3), dtype=np.float64),
                    )
                    self._publish_status(
                        "REJECTED", reason=mask_result.reason,
                        **frontend_details)
                    return
                mask_pixels = int(np.count_nonzero(mask))
                if mask_pixels < self.min_mask_pixels:
                    self.samples.clear()
                    self._publish_debug(
                        color_message, rgb, mask, aligned_depth, color_info,
                        np.empty((0, 3), dtype=np.float64),
                    )
                    self._publish_status("REJECTED", reason="mask_too_small",
                                         mask_pixels=mask_pixels,
                                         **frontend_details)
                    return
                points_optical = backproject_mask(
                    mask, aligned_depth, color_k, self.min_depth, self.max_depth
                )
                if len(points_optical) < self.min_points:
                    self.samples.clear()
                    self._publish_debug(
                        color_message, rgb, mask, aligned_depth, color_info,
                        np.empty((0, 3), dtype=np.float64),
                    )
                    self._publish_status("REJECTED", reason="too_few_points",
                                         mask_pixels=mask_pixels,
                                         points=int(len(points_optical)),
                                         **frontend_details)
                    return
                points_world = self._to_world(points_optical, color_message.header.stamp)
                estimate = estimate_brick_pose(
                    points_world, self.brick_contract.vertical_height
                )
                self._publish_debug(
                    color_message, rgb, mask, aligned_depth, color_info, points_world
                )
                if not self.sampling_enabled:
                    self.samples.clear()
                    self._publish_status(
                        "WAITING_FOR_VIEWPOINT", points=int(len(points_world)),
                        **frontend_details)
                    return
                self.samples.append(estimate)
                stable, position_spread, yaw_spread = self._stable_pose()
                if stable is None:
                    self._publish_status(
                        "ACCUMULATING", frames=len(self.samples), points=int(len(points_world)),
                        position_spread=position_spread, yaw_spread=yaw_spread,
                        **frontend_details
                    )
                    return
                published_seq = self._publish_pose_and_marker(
                    stable, color_message.header
                )
                self._publish_status(
                    "VALID", frames=len(self.samples), points=int(len(points_world)),
                    position_spread=position_spread, yaw_spread=yaw_spread,
                    x=float(stable[0]), y=float(stable[1]), z=float(stable[2]),
                    yaw=float(stable[3]),
                    measurement_stamp_secs=int(color_message.header.stamp.secs),
                    measurement_stamp_nsecs=int(color_message.header.stamp.nsecs),
                    measurement_stamp=float(color_message.header.stamp.to_sec()),
                    source_seq=int(published_seq),
                    source_image_seq=int(color_message.header.seq),
                    **frontend_details
                )
            except (CvBridgeError, ValueError, tf2_ros.TransformException) as error:
                self.samples.clear()
                self._publish_status("REJECTED", reason=str(error))
                rospy.logwarn_throttle(2.0, "[m1_perception] rejected: %s", error)
            except rospy.ROSException as error:
                # Publishers are closed asynchronously during roslaunch
                # teardown; do not turn an otherwise clean shutdown into a
                # message_filters callback traceback.
                if not rospy.is_shutdown():
                    rospy.logwarn_throttle(2.0, "[m1_perception] ROS publish error: %s", error)


if __name__ == "__main__":
    rospy.init_node("m1_aerial_brick_pose")
    AerialBrickPoseNode()
    rospy.spin()
