#!/usr/bin/env python3
"""Bridge Gazebo P450 ground truth into an unambiguous M1 world TF tree."""

import math

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, quaternion_multiply


class WorldTfBridge:
    def __init__(self):
        self.uav_id = int(rospy.get_param("~uav_id", 1))
        self.world_frame = rospy.get_param("~world_frame", "world").lstrip("/")
        self.base_frame = rospy.get_param(
            "~base_frame", "uav{}/base_link_gt".format(self.uav_id)
        ).lstrip("/")
        self.mount_x = float(rospy.get_param("~mount_x", 0.095))
        self.mount_pitch = float(rospy.get_param("~mount_pitch", 0.35))
        self.sensor_z = float(rospy.get_param("~sensor_z", 0.12))
        self.dynamic_broadcaster = tf2_ros.TransformBroadcaster()
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self._publish_camera_statics()
        topic = "/uav{}/prometheus/ground_truth".format(self.uav_id)
        rospy.Subscriber(topic, Odometry, self._ground_truth_cb, queue_size=20)
        rospy.loginfo("[m1_tf] %s -> %s from %s", self.world_frame, self.base_frame, topic)

    def _camera_transform(self, child):
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = self.base_frame
        transform.child_frame_id = child
        transform.transform.translation.x = self.mount_x + math.sin(self.mount_pitch) * self.sensor_z
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = math.cos(self.mount_pitch) * self.sensor_z
        mount = quaternion_from_euler(0.0, self.mount_pitch, 0.0)
        optical = quaternion_from_euler(-math.pi / 2.0, 0.0, -math.pi / 2.0)
        quaternion = quaternion_multiply(mount, optical)
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        return transform

    def _publish_camera_statics(self):
        transforms = [
            self._camera_transform(
                "uav{}/camera_color_optical_frame".format(self.uav_id)
            ),
            self._camera_transform(
                "uav{}/camera_depth_optical_frame".format(self.uav_id)
            ),
        ]
        self.static_broadcaster.sendTransform(transforms)

    def _ground_truth_cb(self, message):
        transform = TransformStamped()
        # The Gazebo plugin uses simulation-elapsed stamps while the official
        # P450 launch intentionally uses wall time. Reception time is therefore
        # the common clock shared with the D435 image messages.
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = self.world_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = message.pose.pose.position.x
        transform.transform.translation.y = message.pose.pose.position.y
        transform.transform.translation.z = message.pose.pose.position.z
        transform.transform.rotation = message.pose.pose.orientation
        self.dynamic_broadcaster.sendTransform(transform)


if __name__ == "__main__":
    rospy.init_node("m1_world_tf_bridge")
    WorldTfBridge()
    rospy.spin()
