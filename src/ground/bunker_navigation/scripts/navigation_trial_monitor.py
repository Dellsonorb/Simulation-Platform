#!/usr/bin/env python
from __future__ import division

import json
import math
import os
import time

import actionlib
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String

from bunker_navigation.metrics import (evaluate_navigation,
                                       heading_alignment_evidence, pose_error,
                                       relative_transform_error,
                                       normalize_angle)


class NavigationTrialMonitor(object):
    TRACKED_FRAMES = ('aubo_i5_base_link', 'ag95_base_link',
                      'camera_link', 'camera_depth_optical_frame')

    def __init__(self):
        self.scenario_id = rospy.get_param('~scenario_id')
        self.target = (float(rospy.get_param('~goal_x')),
                       float(rospy.get_param('~goal_y')),
                       float(rospy.get_param('~goal_yaw')))
        self.result_file = rospy.get_param('~result_file')
        self.timeout = float(rospy.get_param('~timeout', 90.0))
        self.xy_tolerance = float(rospy.get_param('~xy_tolerance', 0.06))
        self.yaw_tolerance = float(rospy.get_param('~yaw_tolerance', 0.06))
        self.expect_heading_gate = bool(rospy.get_param(
            '~expect_heading_gate', False))
        self.gate_release_bearing = math.radians(float(rospy.get_param(
            '/heading_gate/release_bearing_deg', 30.0)))
        self.listener = tf.TransformListener()
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.get_state = rospy.ServiceProxy('/gazebo/get_model_state',
                                            GetModelState)
        self.latest_odom = None
        self.latest_cmd = None
        self.latest_costmap_received_at = None
        self.latest_gate_state = 'IDLE'
        self.gate_states = []
        self.command_samples = []
        self.record_commands = False
        self.release_bearing = None
        rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid,
                         self.costmap_callback, queue_size=1)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_callback, queue_size=1)
        rospy.Subscriber('/heading_gate/state', String,
                         self.gate_state_callback, queue_size=10)

    def odom_callback(self, message):
        self.latest_odom = message

    def costmap_callback(self, unused_message):
        self.latest_costmap_received_at = rospy.Time.now().to_sec()

    def wait_for_fresh_costmap(self, timeout=10.0, maximum_age=0.5):
        deadline = time.time() + timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            now = rospy.Time.now().to_sec()
            if (self.latest_costmap_received_at is not None and
                    now - self.latest_costmap_received_at <= maximum_age):
                return True
            rospy.rostime.wallsleep(0.05)
        return False

    def cmd_callback(self, message):
        self.latest_cmd = message
        if self.record_commands:
            bearing = None
            if self.latest_odom is not None:
                pose = self.latest_odom.pose.pose
                bearing = normalize_angle(math.atan2(
                    self.target[1] - pose.position.y,
                    self.target[0] - pose.position.x) - self.yaw(pose))
            self.command_samples.append({
                'stamp': rospy.Time.now().to_sec(),
                'state': self.latest_gate_state,
                'bearing': bearing,
                'linear': message.linear.x,
                'angular': message.angular.z})

    def gate_state_callback(self, message):
        previous = self.latest_gate_state
        self.latest_gate_state = message.data
        if not self.record_commands:
            return
        if not self.gate_states or self.gate_states[-1] != message.data:
            self.gate_states.append(message.data)
        if (previous == 'ALIGNING' and message.data == 'PASS_THROUGH' and
                self.latest_odom is not None):
            pose = self.latest_odom.pose.pose
            dx = self.target[0] - pose.position.x
            dy = self.target[1] - pose.position.y
            self.release_bearing = abs(normalize_angle(
                math.atan2(dy, dx) - self.yaw(pose)))

    def transforms(self):
        values = {}
        for frame in self.TRACKED_FRAMES:
            self.listener.waitForTransform('base_link', frame, rospy.Time(0),
                                           rospy.Duration(5.0))
            values[frame] = self.listener.lookupTransform(
                'base_link', frame, rospy.Time(0))
        return values

    @staticmethod
    def yaw(pose):
        q = pose.orientation
        return tf.transformations.euler_from_quaternion(
            (q.x, q.y, q.z, q.w))[2]

    def write(self, result):
        directory = os.path.dirname(self.result_file)
        if directory and not os.path.isdir(directory):
            os.makedirs(directory)
        temporary = self.result_file + '.tmp'
        with open(temporary, 'w') as stream:
            json.dump(result, stream, indent=2, sort_keys=True)
            stream.write('\n')
        os.rename(temporary, self.result_file)
        rospy.loginfo('[NAVIGATION_TRIAL] %s', json.dumps(result,
                                                          sort_keys=True))

    def run(self):
        started = time.time()
        try:
            rospy.wait_for_service('/gazebo/get_model_state', timeout=20.0)
            if not self.client.wait_for_server(rospy.Duration(25.0)):
                raise RuntimeError('move_base action server unavailable')
            # Controllers initialize the arm from the spawned zero state.
            # Capture the TF baseline only after that deterministic settling.
            rospy.rostime.wallsleep(2.0)
            initial_tf = self.transforms()
            if not self.wait_for_fresh_costmap():
                raise RuntimeError('fresh local costmap unavailable')
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'odom'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = self.target[0]
            goal.target_pose.pose.position.y = self.target[1]
            q = tf.transformations.quaternion_from_euler(0, 0, self.target[2])
            goal.target_pose.pose.orientation.x = q[0]
            goal.target_pose.pose.orientation.y = q[1]
            goal.target_pose.pose.orientation.z = q[2]
            goal.target_pose.pose.orientation.w = q[3]
            navigation_started = time.time()
            initial_pose = self.latest_odom.pose.pose
            initial_bearing = normalize_angle(
                math.atan2(self.target[1] - initial_pose.position.y,
                           self.target[0] - initial_pose.position.x) -
                self.yaw(initial_pose))
            self.gate_states = []
            self.command_samples = []
            self.release_bearing = None
            self.record_commands = True
            self.client.send_goal(goal)
            deadline = time.time() + self.timeout
            while (not rospy.is_shutdown() and time.time() < deadline and
                   not self.client.wait_for_result(rospy.Duration(0.2))):
                pass
            if self.client.get_state() in (GoalStatus.PENDING,
                                            GoalStatus.ACTIVE):
                self.client.cancel_goal()
                rospy.rostime.wallsleep(0.2)
            action_state = self.client.get_state()
            navigation_duration = time.time() - navigation_started
            # Observe move_base's terminal command; the harness must never
            # manufacture a stop command itself.
            rospy.rostime.wallsleep(0.3)
            self.record_commands = False
            state = self.get_state('bunker_aubo', 'world')
            if not state.success:
                raise RuntimeError('Gazebo model state unavailable: ' +
                                   state.status_message)
            actual = (state.pose.position.x, state.pose.position.y,
                      self.yaw(state.pose))
            odom_actual = None
            if self.latest_odom is not None:
                odom_actual = {
                    'x': self.latest_odom.pose.pose.position.x,
                    'y': self.latest_odom.pose.pose.position.y,
                    'yaw': self.yaw(self.latest_odom.pose.pose)}
            error = pose_error(actual, self.target)
            final_tf = self.transforms()
            tf_errors = dict((frame, relative_transform_error(
                initial_tf[frame], final_tf[frame]))
                for frame in self.TRACKED_FRAMES)
            tf_preserved = all(
                value['translation'] <= 1e-6 and value['rotation'] <= 1e-6
                for value in tf_errors.values())
            stopped_twist = (self.latest_cmd if self.latest_cmd is not None
                             else state.twist)
            linear_speed = math.sqrt(stopped_twist.linear.x ** 2 +
                                     stopped_twist.linear.y ** 2)
            verdict = evaluate_navigation(
                action_state == GoalStatus.SUCCEEDED, error['xy'], error['yaw'],
                linear_speed, stopped_twist.angular.z, tf_preserved,
                self.xy_tolerance, self.yaw_tolerance, 0.01, 0.01)
            gate_seen = 'ALIGNING' in self.gate_states
            evidence = heading_alignment_evidence(
                self.command_samples, initial_bearing,
                self.gate_release_bearing)
            max_alignment_linear = evidence['max_linear']
            first_alignment_angular = evidence['first_angular']
            correct_turn_direction = evidence['correct_turn_direction']
            gate_ok = (not self.expect_heading_gate or
                       (gate_seen and max_alignment_linear <= 1e-6 and
                        correct_turn_direction and
                        self.release_bearing is not None and
                        self.release_bearing <= math.radians(31.0)))
            if verdict['success'] and not gate_ok:
                verdict = {'success': False,
                           'failure_type': 'heading_gate_invalid'}
            result = {
                'scenario_id': self.scenario_id,
                'target': {'x': self.target[0], 'y': self.target[1],
                           'yaw': self.target[2]},
                'actual': {'x': actual[0], 'y': actual[1], 'yaw': actual[2]},
                'odom_actual': odom_actual,
                'action_state': action_state,
                'xy_error': error['xy'], 'yaw_error': error['yaw'],
                'linear_speed': linear_speed,
                'angular_speed': abs(stopped_twist.angular.z),
                'tf_preserved': tf_preserved, 'tf_errors': tf_errors,
                'heading_gate_expected': self.expect_heading_gate,
                'heading_gate_seen': gate_seen,
                'gate_state_sequence': self.gate_states,
                'alignment_max_linear': max_alignment_linear,
                'alignment_first_angular': first_alignment_angular,
                'alignment_sample_count': evidence['sample_count'],
                'alignment_correct_turn_direction': correct_turn_direction,
                'alignment_release_bearing': self.release_bearing,
                'duration': navigation_duration,
                'wall_duration': time.time() - started,
                'success': verdict['success'],
                'failure_type': verdict['failure_type'],
            }
            self.write(result)
            return 0 if result['success'] else 2
        except Exception as error:
            self.write({
                'scenario_id': self.scenario_id, 'target': {
                    'x': self.target[0], 'y': self.target[1],
                    'yaw': self.target[2]},
                'success': False, 'failure_type': 'harness_failed',
                'failure_detail': str(error),
                'xy_error': float('inf'), 'yaw_error': float('inf'),
                'duration': time.time() - started})
            return 3


if __name__ == '__main__':
    rospy.init_node('navigation_trial_monitor')
    raise SystemExit(NavigationTrialMonitor().run())
