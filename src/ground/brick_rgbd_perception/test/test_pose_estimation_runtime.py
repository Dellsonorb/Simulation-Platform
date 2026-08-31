#!/usr/bin/env python
from __future__ import division

import math
import threading
import unittest

import numpy as np
import rospy
import rostest
import tf.transformations as transformations
import tf2_ros
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray

from brick_rgbd_perception.pose_estimation import yaw_error_mod_pi


class PoseEstimationRuntimeTest(unittest.TestCase):
    def test_multiframe_error_against_gazebo_ground_truth(self):
        tf_buffer = tf2_ros.Buffer(rospy.Duration(30.0))
        tf2_ros.TransformListener(tf_buffer)
        rospy.wait_for_service('/gazebo/get_model_state', timeout=20.0)
        get_model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        samples = []
        lock = threading.Lock()

        def callback(message):
            brick = get_model('brick', 'world')
            if not brick.success:
                return
            with lock:
                if len(samples) < 30:
                    samples.append((message, brick.pose))

        subscriber = rospy.Subscriber(
            '/brick_pose_estimator/pose_debug', PoseStamped, callback,
            queue_size=30)
        deadline = rospy.Time.now() + rospy.Duration(100.0)
        while not rospy.is_shutdown():
            with lock:
                count = len(samples)
            if count >= 30 or rospy.Time.now() >= deadline:
                break
            rospy.sleep(0.05)
        subscriber.unregister()
        self.assertEqual(30, len(samples))
        markers = rospy.wait_for_message(
            '/brick_pose_estimator/markers', MarkerArray, timeout=5.0)
        self.assertGreaterEqual(len(markers.markers), 2)

        position_errors = []
        xy_errors = []
        z_errors = []
        yaw_errors = []
        for estimate, brick_pose in samples:
            self.assertEqual('aubo_i5_base_link', estimate.header.frame_id)
            transform = tf_buffer.lookup_transform(
                estimate.header.frame_id, 'world', estimate.header.stamp,
                rospy.Duration(5.0)).transform
            base_from_world = np.dot(
                transformations.translation_matrix((
                    transform.translation.x, transform.translation.y,
                    transform.translation.z)),
                transformations.quaternion_matrix((
                    transform.rotation.x, transform.rotation.y,
                    transform.rotation.z, transform.rotation.w)))
            world_from_brick = np.dot(
                transformations.translation_matrix((
                    brick_pose.position.x, brick_pose.position.y,
                    brick_pose.position.z)),
                transformations.quaternion_matrix((
                    brick_pose.orientation.x, brick_pose.orientation.y,
                    brick_pose.orientation.z, brick_pose.orientation.w)))
            base_from_brick = np.dot(base_from_world, world_from_brick)
            truth_position = base_from_brick[:3, 3]
            truth_yaw = math.atan2(base_from_brick[1, 0],
                                   base_from_brick[0, 0])
            estimate_position = np.array((
                estimate.pose.position.x, estimate.pose.position.y,
                estimate.pose.position.z))
            estimate_rotation = transformations.quaternion_matrix((
                estimate.pose.orientation.x, estimate.pose.orientation.y,
                estimate.pose.orientation.z, estimate.pose.orientation.w))
            estimate_yaw = math.atan2(estimate_rotation[1, 0],
                                      estimate_rotation[0, 0])
            delta = estimate_position - truth_position
            position_errors.append(np.linalg.norm(delta))
            xy_errors.append(np.linalg.norm(delta[:2]))
            z_errors.append(abs(delta[2]))
            yaw_errors.append(yaw_error_mod_pi(estimate_yaw, truth_yaw))

        arrays = [np.asarray(values) for values in (
            position_errors, xy_errors, z_errors, yaw_errors)]
        rospy.loginfo(
            '4DoF 30-frame errors: position mean/max=%.6f/%.6f m, '
            'XY mean/max=%.6f/%.6f m, Z mean/max=%.6f/%.6f m, '
            'yaw mean/max=%.6f/%.6f rad',
            arrays[0].mean(), arrays[0].max(),
            arrays[1].mean(), arrays[1].max(),
            arrays[2].mean(), arrays[2].max(),
            arrays[3].mean(), arrays[3].max())
        self.assertLess(arrays[0].mean(), 0.003)
        self.assertLess(arrays[0].max(), 0.005)
        self.assertLess(arrays[1].mean(), 0.003)
        self.assertLess(arrays[1].max(), 0.005)
        self.assertLess(arrays[2].mean(), 0.001)
        self.assertLess(arrays[2].max(), 0.002)
        self.assertLess(arrays[3].mean(), 0.010)
        self.assertLess(arrays[3].max(), 0.020)
        self.assertNotIn('/brick_pose', dict(rospy.get_published_topics()))


if __name__ == '__main__':
    rospy.init_node('test_pose_estimation_runtime')
    rostest.rosrun('brick_rgbd_perception', 'pose_estimation_runtime',
                   PoseEstimationRuntimeTest)
