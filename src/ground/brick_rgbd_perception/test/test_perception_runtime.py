#!/usr/bin/env python
from __future__ import division

import unittest
import time

import numpy as np
import rospy
import rostest
import sensor_msgs.point_cloud2 as point_cloud2
import tf.transformations as transformations
import tf2_ros
from cv_bridge import CvBridge
from gazebo_msgs.srv import GetModelState
from sensor_msgs.msg import Image, PointCloud2


class PerceptionRuntimeTest(unittest.TestCase):
    def test_mask_and_cloud_are_brick_only(self):
        tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(tf_buffer)
        deadline = time.time() + 80.0
        mask_msg = None
        mask = None
        while time.time() < deadline:
            mask_msg = rospy.wait_for_message(
                '/brick_rgbd_perception/mask', Image, timeout=10.0)
            mask = CvBridge().imgmsg_to_cv2(mask_msg,
                                             desired_encoding='mono8')
            if np.count_nonzero(mask) > 60000:
                break
        self.assertIsNotNone(mask_msg)
        self.assertGreater(np.count_nonzero(mask), 60000)
        debug_msg = rospy.wait_for_message(
            '/brick_rgbd_perception/debug_rgb', Image, timeout=10.0)
        cloud_msg = None
        while time.time() < deadline:
            candidate = rospy.wait_for_message(
                '/brick_rgbd_perception/points', PointCloud2, timeout=10.0)
            if candidate.width > 60000:
                cloud_msg = candidate
                break
        self.assertIsNotNone(cloud_msg)
        self.assertEqual('mono8', mask_msg.encoding)
        self.assertEqual('rgb8', debug_msg.encoding)
        self.assertEqual('camera_depth_optical_frame', cloud_msg.header.frame_id)

        points = np.asarray(list(point_cloud2.read_points(
            cloud_msg, field_names=('x', 'y', 'z'), skip_nans=True)),
                            dtype=np.float64)
        self.assertGreater(points.shape[0], 100)
        self.assertTrue(np.all(np.isfinite(points)))
        self.assertGreaterEqual(points[:, 2].min(), 0.15)
        self.assertLessEqual(points[:, 2].max(), 2.0)

        transform = tf_buffer.lookup_transform(
            'world', cloud_msg.header.frame_id, cloud_msg.header.stamp,
            rospy.Duration(10.0)).transform
        translation = transformations.translation_matrix((
            transform.translation.x, transform.translation.y,
            transform.translation.z))
        rotation = transformations.quaternion_matrix((
            transform.rotation.x, transform.rotation.y,
            transform.rotation.z, transform.rotation.w))
        homogeneous = np.column_stack((points, np.ones(points.shape[0])))
        world_points = np.dot(translation, np.dot(rotation, homogeneous.T)).T[:, :3]

        rospy.wait_for_service('/gazebo/get_model_state', timeout=10.0)
        brick = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)(
            'brick', 'world')
        self.assertTrue(brick.success)
        brick_translation = transformations.translation_matrix((
            brick.pose.position.x, brick.pose.position.y, brick.pose.position.z))
        brick_rotation = transformations.quaternion_matrix((
            brick.pose.orientation.x, brick.pose.orientation.y,
            brick.pose.orientation.z, brick.pose.orientation.w))
        brick_from_world = np.linalg.inv(np.dot(brick_translation, brick_rotation))
        local = np.dot(
            brick_from_world,
            np.column_stack((world_points, np.ones(world_points.shape[0]))).T
        ).T[:, :3]
        inside = ((np.abs(local[:, 0]) <= 0.120 + 0.015) &
                  (np.abs(local[:, 1]) <= 0.0575 + 0.015) &
                  (np.abs(local[:, 2]) <= 0.0265 + 0.015))
        self.assertGreater(np.mean(inside), 0.98)

        published = dict(rospy.get_published_topics())
        self.assertNotIn('/brick_pose', published)
        rospy.loginfo('verified brick cloud: points=%d depth=[%.4f, %.4f] m '
                      'inside_brick=%.2f%%', points.shape[0],
                      points[:, 2].min(), points[:, 2].max(),
                      100.0 * np.mean(inside))


if __name__ == '__main__':
    rospy.init_node('test_perception_runtime')
    rostest.rosrun('brick_rgbd_perception', 'perception_runtime',
                   PerceptionRuntimeTest)
