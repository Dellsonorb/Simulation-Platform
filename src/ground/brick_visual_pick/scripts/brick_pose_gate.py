#!/usr/bin/env python
from __future__ import division

import math
import threading

import rospy
import tf.transformations as transformations
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from brick_visual_pick.quality_gate import evaluate_pose_samples


def isfinite(value):
    return not math.isnan(value) and not math.isinf(value)


class BrickPoseGate(object):
    def __init__(self):
        self.required_frame = rospy.get_param('~required_frame')
        self.required_samples = int(rospy.get_param('~required_samples', 6))
        self.minimum_span = float(rospy.get_param('~minimum_sample_span', 0.30))
        self.position_tolerance = float(
            rospy.get_param('~position_stability_tolerance', 0.003))
        self.yaw_tolerance = float(
            rospy.get_param('~yaw_stability_tolerance', 0.02))
        self.joint_names = rospy.get_param('~joint_names')
        self.observation_positions = rospy.get_param('~positions')
        self.joint_tolerance = float(
            rospy.get_param('~observation_joint_tolerance', 0.08))
        self.observation_timeout = float(
            rospy.get_param('~observation_timeout', 35.0))
        self.perception_timeout = float(
            rospy.get_param('~perception_timeout', 15.0))
        self.estimate_max_age = float(rospy.get_param('~estimate_max_age', 1.0))
        self.lock = threading.RLock()
        self.started_at = rospy.Time.now()
        self.observation_reached_at = None
        self.samples = []
        self.done = False
        self.last_reason = 'no_estimate'

        self.pose_pub = rospy.Publisher(
            rospy.get_param('~output_topic', '/brick_pose'), PoseStamped,
            queue_size=1, latch=True)
        self.status_pub = rospy.Publisher(
            rospy.get_param('~status_topic', '/visual_pick/status'), String,
            queue_size=1, latch=True)
        rospy.Subscriber('/joint_states', JointState, self.joint_callback,
                         queue_size=1)
        rospy.Subscriber(
            rospy.get_param('~estimate_topic',
                            '/brick_pose_estimator/pose_debug'),
            PoseStamped, self.pose_callback, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        self.publish_status('WAITING_OBSERVATION')

    def publish_status(self, value):
        self.status_pub.publish(String(data=value))
        rospy.loginfo('[VISUAL_PICK_GATE] %s', value)

    def reject(self, reason):
        with self.lock:
            if self.done:
                return
            self.done = True
            self.publish_status('PERCEPTION_REJECTED:' + reason)
            rospy.logerr('[VISUAL_PICK_GATE] no /brick_pose published: %s', reason)

    def joint_callback(self, message):
        with self.lock:
            if self.done or self.observation_reached_at is not None:
                return
            measured = dict(zip(message.name, message.position))
            if any(name not in measured for name in self.joint_names):
                return
            maximum_error = max(
                abs(measured[name] - target)
                for name, target in zip(self.joint_names,
                                        self.observation_positions))
            if maximum_error <= self.joint_tolerance:
                self.observation_reached_at = rospy.Time.now()
                self.samples = []
                self.publish_status('WAITING_ESTIMATE')
                rospy.loginfo('[VISUAL_PICK_GATE] observation pose verified; '
                              'maximum joint error %.5f rad', maximum_error)

    def pose_callback(self, message):
        with self.lock:
            if self.done or self.observation_reached_at is None:
                return
            stamp = message.header.stamp.to_sec()
            now = rospy.Time.now().to_sec()
            values = (stamp, message.pose.position.x,
                      message.pose.position.y, message.pose.position.z,
                      message.pose.orientation.x, message.pose.orientation.y,
                      message.pose.orientation.z, message.pose.orientation.w)
            if not all(isfinite(value) for value in values):
                self.last_reason = 'non_finite_message'
                return
            if stamp <= self.observation_reached_at.to_sec():
                self.last_reason = 'estimate_before_observation'
                return
            if now - stamp < 0.0 or now - stamp > self.estimate_max_age:
                self.last_reason = 'stale_estimate'
                return
            quaternion = message.pose.orientation
            quaternion_norm = math.sqrt(
                quaternion.x ** 2 + quaternion.y ** 2 +
                quaternion.z ** 2 + quaternion.w ** 2)
            if quaternion_norm < 0.99 or quaternion_norm > 1.01:
                self.last_reason = 'invalid_quaternion'
                return
            yaw = transformations.euler_from_quaternion((
                quaternion.x, quaternion.y, quaternion.z, quaternion.w))[2]
            self.samples.append({
                'stamp': stamp,
                'frame_id': message.header.frame_id,
                'x': message.pose.position.x,
                'y': message.pose.position.y,
                'z': message.pose.position.z,
                'yaw': yaw,
            })
            self.samples = self.samples[-max(30, self.required_samples):]
            result = evaluate_pose_samples(
                self.samples, self.required_samples, self.minimum_span,
                self.position_tolerance, self.yaw_tolerance,
                self.required_frame)
            self.last_reason = result['reason']
            if not result['accepted']:
                return

            accepted = result['pose']
            output = PoseStamped()
            output.header.stamp = message.header.stamp
            output.header.frame_id = self.required_frame
            output.pose.position.x = accepted['x']
            output.pose.position.y = accepted['y']
            output.pose.position.z = accepted['z']
            quaternion = transformations.quaternion_from_euler(
                0.0, 0.0, accepted['yaw'])
            (output.pose.orientation.x, output.pose.orientation.y,
             output.pose.orientation.z, output.pose.orientation.w) = quaternion
            self.pose_pub.publish(output)
            self.done = True
            self.publish_status('POSE_PUBLISHED')
            rospy.loginfo('[VISUAL_PICK_GATE] accepted %d estimates over %.3f s; '
                          'position spread %.6f m, yaw spread %.6f rad',
                          result['sample_count'], result['time_span'],
                          result['position_spread'], result['yaw_spread'])

    def timer_callback(self, _event):
        with self.lock:
            if self.done:
                return
            now = rospy.Time.now()
            if self.observation_reached_at is None:
                if (now - self.started_at).to_sec() > self.observation_timeout:
                    self.reject('observation_pose_timeout')
            elif ((now - self.observation_reached_at).to_sec() >
                  self.perception_timeout):
                self.reject('timeout_' + self.last_reason)


if __name__ == '__main__':
    rospy.init_node('brick_pose_gate')
    BrickPoseGate()
    rospy.spin()
