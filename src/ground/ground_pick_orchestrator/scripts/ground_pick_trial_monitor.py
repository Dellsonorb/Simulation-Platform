#!/usr/bin/env python
from __future__ import division

import json
import math
import os
import time

import rospy
import tf.transformations as transformations
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String

from ground_pick_orchestrator.safety_guards import terminal_stop_is_safe


def as_bool(value):
    return value if isinstance(value, bool) else str(value).lower() == 'true'


class TrialMonitor(object):
    def __init__(self):
        self.scenario_id = rospy.get_param('~scenario_id')
        self.result_file = rospy.get_param('~result_file')
        self.known = (float(rospy.get_param('~known_x')),
                      float(rospy.get_param('~known_y')),
                      float(rospy.get_param('~known_yaw')))
        self.expect_success = as_bool(rospy.get_param('~expect_success'))
        self.expected_failure = rospy.get_param('~expected_failure', '')
        self.timeout = float(rospy.get_param('~timeout', 150.0))
        self.status = None
        self.history = []
        self.latest_cmd = None
        self.latest_cmd_wall = None
        self.terminal_wall = None
        self.formal_stamps = []
        self.get_state = rospy.ServiceProxy('/gazebo/get_model_state',
                                            GetModelState)
        self.publisher = rospy.Publisher('/ground_pick/known_brick_pose',
                                         PoseStamped, queue_size=1, latch=True)
        rospy.Subscriber('/ground_pick/status', String, self.status_callback,
                         queue_size=30)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_callback, queue_size=5)
        rospy.Subscriber('/brick_pose', PoseStamped, self.pose_callback,
                         queue_size=5)

    def status_callback(self, message):
        try:
            self.status = json.loads(message.data)
        except ValueError:
            return
        state = self.status.get('state')
        if state in ('SUCCEEDED', 'FAILED') and self.terminal_wall is None:
            self.terminal_wall = time.time()
        if not self.history or self.history[-1] != state:
            self.history.append(state)

    def cmd_callback(self, message):
        self.latest_cmd = message
        self.latest_cmd_wall = time.time()

    def pose_callback(self, message):
        self.formal_stamps.append(message.header.stamp.to_sec())

    def height(self):
        try:
            value = self.get_state('brick', 'world')
            return value.pose.position.z if value.success else None
        except rospy.ServiceException:
            return None

    def model_motion_sample(self):
        try:
            value = self.get_state('bunker_aubo', 'world')
            if not value.success:
                return None
            quaternion = value.pose.orientation
            yaw = transformations.euler_from_quaternion((
                quaternion.x, quaternion.y, quaternion.z,
                quaternion.w))[2]
            return (value.pose.position.x, value.pose.position.y, yaw,
                    value.twist.linear.x, value.twist.linear.y,
                    value.twist.angular.z)
        except rospy.ServiceException:
            return None

    def terminal_stop_evidence(self):
        deadline = time.time() + 4.0
        first = None
        second = None
        sample_span = 0.0
        while not rospy.is_shutdown() and time.time() < deadline:
            command_fresh = (
                self.latest_cmd is not None and
                self.latest_cmd_wall is not None and
                self.terminal_wall is not None and
                self.latest_cmd_wall >= self.terminal_wall)
            if command_fresh:
                first = self.model_motion_sample()
                if first is not None:
                    first_wall = time.time()
                    rospy.rostime.wallsleep(0.5)
                    second = self.model_motion_sample()
                    sample_span = time.time() - first_wall
                    if second is not None:
                        break
            rospy.rostime.wallsleep(0.05)
        command_motion = None if self.latest_cmd is None else (
            self.latest_cmd.linear.x, self.latest_cmd.angular.z)
        stop_command_fresh = bool(
            command_motion is not None and
            self.latest_cmd_wall is not None and
            self.terminal_wall is not None and
            self.latest_cmd_wall >= self.terminal_wall and
            abs(command_motion[0]) <= 0.01 and
            abs(command_motion[1]) <= 0.01)
        model_stationary = terminal_stop_is_safe(
            (0.0, 0.0), True, first, second, sample_span, 0.01, 0.02)
        safe_stop = terminal_stop_is_safe(
            command_motion, stop_command_fresh, first, second, sample_span,
            0.01, 0.02)
        return stop_command_fresh, model_stationary, safe_stop

    def write(self, result):
        directory = os.path.dirname(self.result_file)
        if directory and not os.path.isdir(directory):
            os.makedirs(directory)
        temporary = self.result_file + '.tmp'
        with open(temporary, 'w') as stream:
            json.dump(result, stream, indent=2, sort_keys=True)
            stream.write('\n')
        os.rename(temporary, self.result_file)

    def run(self):
        started = time.time()
        rospy.wait_for_service('/gazebo/get_model_state', timeout=25.0)
        deadline = time.time() + 30.0
        initial = None
        while not rospy.is_shutdown() and time.time() < deadline:
            initial = self.height()
            if initial is not None and self.status and \
                    self.status.get('state') == 'IDLE':
                break
            rospy.rostime.wallsleep(0.1)
        rospy.rostime.wallsleep(2.0)
        message = PoseStamped()
        message.header.frame_id = 'odom'
        message.header.stamp = rospy.Time.now()
        message.pose.position.x = self.known[0]
        message.pose.position.y = self.known[1]
        quaternion = transformations.quaternion_from_euler(
            0.0, 0.0, self.known[2])
        (message.pose.orientation.x, message.pose.orientation.y,
         message.pose.orientation.z, message.pose.orientation.w) = quaternion
        self.publisher.publish(message)
        deadline = time.time() + self.timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.status and self.status.get('state') in ('SUCCEEDED',
                                                            'FAILED'):
                break
            rospy.rostime.wallsleep(0.1)
        status = self.status or {'state': 'TIMEOUT', 'failure_type':
                                 'harness_timeout'}
        stop_command_fresh, model_stationary, stopped = \
            self.terminal_stop_evidence()
        outcome_ok = (status.get('state') == 'SUCCEEDED') \
            if self.expect_success else (
                status.get('state') == 'FAILED' and
                status.get('failure_type') == self.expected_failure)
        safe_stop = stopped and (self.expect_success or
                                 not status.get('end_to_end_success', False))
        result = dict(status)
        result.update({
            'scenario_id': self.scenario_id,
            'expect_success': self.expect_success,
            'expected_failure': self.expected_failure,
            'state_history': self.history,
            'formal_pose_count': len(self.formal_stamps),
            'safe_stop': safe_stop,
            'stop_command_fresh': stop_command_fresh,
            'model_stationary': model_stationary,
            'duration': time.time() - started,
            'monitor_passed': bool(outcome_ok and safe_stop),
            'measured_brick_z': (self.height()
                                 if status.get('state') in ('SUCCEEDED',
                                                            'FAILED')
                                 else None),
        })
        self.write(result)
        return 0 if result['monitor_passed'] else 1


if __name__ == '__main__':
    rospy.init_node('ground_pick_trial_monitor')
    raise SystemExit(TrialMonitor().run())
