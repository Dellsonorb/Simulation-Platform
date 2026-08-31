#!/usr/bin/env python
from __future__ import division

import math
import time
import unittest

import actionlib
import numpy as np
import rospy
import rostest
import tf2_geometry_msgs
import tf2_ros
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image, JointState
from trajectory_msgs.msg import JointTrajectoryPoint


class RgbdCameraTest(unittest.TestCase):
    joint_names = [
        'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
        'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    observation = [0.2040, -0.9584, 1.7778, 1.1465, 1.5710, 0.2038]

    def collect(self, timeout=20.0):
        messages = {}

        def subscribe(topic, message_type, key):
            return rospy.Subscriber(
                topic, message_type,
                lambda message: messages.setdefault(key, message), queue_size=1)

        subscribers = [
            subscribe('/camera/color/image_raw', Image, 'rgb'),
            subscribe('/camera/color/camera_info', CameraInfo, 'color_info'),
            subscribe('/camera/depth/image_rect_raw', Image, 'depth'),
            subscribe('/camera/depth/camera_info', CameraInfo, 'depth_info'),
            subscribe('/gazebo/model_states', ModelStates, 'models')]
        deadline = time.time() + timeout
        while len(messages) < 5 and time.time() < deadline:
            time.sleep(0.02)
        self.assertEqual(5, len(messages), 'missing RGB-D messages: ' +
                         str(sorted(messages.keys())))
        return messages, subscribers

    def wait_for_observation(self, timeout=30.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            message = rospy.wait_for_message('/joint_states', JointState, timeout=5.0)
            positions = dict(zip(message.name, message.position))
            if all(name in positions for name in self.joint_names):
                error = max(abs(positions[name] - target)
                            for name, target in zip(self.joint_names,
                                                    self.observation))
                if error < 0.08:
                    return
        self.fail('AUBO did not reach the D435 observation pose')

    def move(self, client, positions):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(3.0)
        goal.trajectory.points = [point]
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.2)
        client.send_goal(goal)
        client.wait_for_result()
        self.assertEqual(GoalStatus.SUCCEEDED, client.get_state())

    @staticmethod
    def translation(transform):
        value = transform.transform.translation
        return np.array([value.x, value.y, value.z])

    def test_rgbd_visibility_tf_and_eye_in_hand_motion(self):
        self.wait_for_observation()
        messages, subscribers = self.collect()
        bridge = CvBridge()
        rgb = bridge.imgmsg_to_cv2(messages['rgb'], 'rgb8')
        depth = bridge.imgmsg_to_cv2(messages['depth'], 'passthrough')
        self.assertEqual('rgb8', messages['rgb'].encoding)
        self.assertEqual((480, 640, 3), rgb.shape)
        self.assertGreater(float(rgb.std()), 1.0)
        self.assertEqual('camera_color_optical_frame',
                         messages['rgb'].header.frame_id)
        self.assertEqual('32FC1', messages['depth'].encoding)
        self.assertEqual((480, 640), depth.shape)
        finite = np.isfinite(depth) & (depth > 0.1) & (depth < 10.0)
        self.assertGreater(float(finite.mean()), 0.05)
        self.assertEqual('camera_depth_optical_frame',
                         messages['depth'].header.frame_id)

        color_info = messages['color_info']
        depth_info = messages['depth_info']
        for info, frame in ((color_info, 'camera_color_optical_frame'),
                            (depth_info, 'camera_depth_optical_frame')):
            self.assertEqual((640, 480), (info.width, info.height))
            self.assertGreater(info.K[0], 0.0)
            self.assertGreater(info.K[4], 0.0)
            self.assertEqual(frame, info.header.frame_id)

        buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buffer)
        time.sleep(1.0)
        models = messages['models']
        brick_pose = models.pose[models.name.index('brick')]
        brick = PointStamped()
        brick.header.frame_id = 'world'
        brick.header.stamp = rospy.Time(0)
        brick.point = brick_pose.position
        optical = buffer.transform(
            brick, 'camera_depth_optical_frame', rospy.Duration(5.0))
        self.assertGreater(optical.point.z, 0.0)
        u = depth_info.K[0] * optical.point.x / optical.point.z + depth_info.K[2]
        v = depth_info.K[4] * optical.point.y / optical.point.z + depth_info.K[5]
        self.assertTrue(0.0 <= u < depth_info.width)
        self.assertTrue(0.0 <= v < depth_info.height)
        ui, vi = int(round(u)), int(round(v))
        patch = depth[max(0, vi - 4):vi + 5, max(0, ui - 4):ui + 5]
        valid = patch[np.isfinite(patch) & (patch > 0.0)]
        self.assertGreater(valid.size, 0)
        self.assertLess(abs(float(np.median(valid)) - optical.point.z), 0.08)

        client = actionlib.SimpleActionClient(
            '/arm_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction)
        client.wait_for_server()
        before = buffer.lookup_transform(
            'world', 'camera_depth_optical_frame', rospy.Time(0),
            rospy.Duration(5.0))
        stamp_before = messages['depth'].header.stamp
        moved = list(self.observation)
        moved[0] += 0.1
        try:
            self.move(client, moved)
            time.sleep(0.5)
            after = buffer.lookup_transform(
                'world', 'camera_depth_optical_frame', rospy.Time(0),
                rospy.Duration(5.0))
            newer = rospy.wait_for_message(
                '/camera/depth/image_rect_raw', Image, timeout=5.0)
            self.assertGreater(
                np.linalg.norm(self.translation(after) - self.translation(before)),
                0.01)
            self.assertGreater(newer.header.stamp, stamp_before)
        finally:
            self.move(client, self.observation)


if __name__ == '__main__':
    rospy.init_node('test_rgbd_camera')
    rostest.rosrun('bunker_aubo_gazebo', 'test_rgbd_camera', RgbdCameraTest)
