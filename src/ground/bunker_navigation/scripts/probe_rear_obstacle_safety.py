#!/usr/bin/env python
"""Verify rear-goal alignment against an obstacle seen in local costmap."""
from __future__ import division

import json
import math
import os
import time

import actionlib
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String


def polygon(center_x, center_y, yaw, half_x, half_y):
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return [(center_x + cosine * x - sine * y,
             center_y + sine * x + cosine * y)
            for x, y in ((-half_x, -half_y), (-half_x, half_y),
                         (half_x, half_y), (half_x, -half_y))]


def overlaps(first, second):
    for shape in (first, second):
        for index in range(len(shape)):
            start = shape[index]
            end = shape[(index + 1) % len(shape)]
            axis = (-(end[1] - start[1]), end[0] - start[0])
            first_projection = [x * axis[0] + y * axis[1]
                                for x, y in first]
            second_projection = [x * axis[0] + y * axis[1]
                                 for x, y in second]
            if (max(first_projection) < min(second_projection) or
                    max(second_projection) < min(first_projection)):
                return False
    return True


class RearObstacleSafetyProbe(object):
    def __init__(self):
        self.result_file = rospy.get_param(
            '~result_file', '/tmp/rear_obstacle_safety.json')
        self.obstacle_y = float(rospy.get_param('~obstacle_y', 0.58))
        self.expect_blocked = bool(rospy.get_param('~expect_blocked', True))
        self.odom = None
        self.state = 'IDLE'
        self.command = (0.0, 0.0)
        self.overlap_samples = []
        self.alignment_samples = 0
        self.obstacle_spawned = False
        self.obstacle_in_costmap = False
        self.obstacle_cost = -1
        self.state_history = []
        self.command_history = []
        self.tracking_started = False
        rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber('/cmd_vel', Twist, self.command_callback, queue_size=1)
        rospy.Subscriber('/heading_gate/state', String,
                         self.state_callback, queue_size=1)
        rospy.Subscriber('/gazebo/model_states', ModelStates,
                         self.model_states_callback, queue_size=1)
        rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid,
                         self.costmap_callback, queue_size=1)
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

    @staticmethod
    def yaw(pose):
        orientation = pose.orientation
        return tf.transformations.euler_from_quaternion((
            orientation.x, orientation.y, orientation.z,
            orientation.w))[2]

    def odom_callback(self, message):
        self.odom = message
        if not self.tracking_started:
            return
        pose = message.pose.pose
        yaw = self.yaw(pose)
        robot = polygon(pose.position.x, pose.position.y, yaw, 0.52, 0.39)
        obstacle = polygon(0.0, self.obstacle_y, 0.0, 0.10, 0.10)
        if self.state == 'ALIGNING':
            self.alignment_samples += 1
        if overlaps(robot, obstacle):
            self.overlap_samples.append({
                'stamp': rospy.Time.now().to_sec(),
                'x': pose.position.x, 'y': pose.position.y, 'yaw': yaw,
                'linear': self.command[0], 'angular': self.command[1],
                'gate_state': self.state})

    def command_callback(self, message):
        self.command = (message.linear.x, message.angular.z)
        self.command_history.append((rospy.Time.now().to_sec(), self.state,
                                     message.linear.x, message.angular.z))

    def state_callback(self, message):
        self.state = message.data
        self.state_history.append((rospy.Time.now().to_sec(), message.data))

    def model_states_callback(self, message):
        self.obstacle_spawned = 'rear_gate_obstacle' in message.name

    def costmap_callback(self, message):
        resolution = message.info.resolution
        if resolution <= 0.0:
            return
        minimum_mx = int(math.floor(
            (-0.15 - message.info.origin.position.x) / resolution))
        maximum_mx = int(math.floor(
            (0.15 - message.info.origin.position.x) / resolution))
        minimum_my = int(math.floor(
            (self.obstacle_y - 0.15 - message.info.origin.position.y) /
            resolution))
        maximum_my = int(math.floor(
            (self.obstacle_y + 0.15 - message.info.origin.position.y) /
            resolution))
        if (minimum_mx < 0 or minimum_my < 0 or
                maximum_mx >= message.info.width or
                maximum_my >= message.info.height):
            return
        self.obstacle_cost = max(
            message.data[my * message.info.width + mx]
            for my in range(minimum_my, maximum_my + 1)
            for mx in range(minimum_mx, maximum_mx + 1))
        self.obstacle_in_costmap = self.obstacle_cost >= 100

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
        if not self.client.wait_for_server(rospy.Duration(25.0)):
            raise RuntimeError('move_base action server unavailable')
        deadline = time.time() + 10.0
        while ((self.odom is None or not self.obstacle_spawned or
                not self.obstacle_in_costmap) and
               time.time() < deadline):
            rospy.sleep(0.05)
        if self.odom is None:
            raise RuntimeError('odometry unavailable')
        if not self.obstacle_spawned:
            raise RuntimeError('Gazebo obstacle was not spawned')
        if not self.obstacle_in_costmap:
            raise RuntimeError(
                'Gazebo obstacle did not enter local costmap (cost=%s)' %
                self.obstacle_cost)
        initial = self.odom.pose.pose
        initial_overlap = overlaps(
            polygon(initial.position.x, initial.position.y,
                    self.yaw(initial), 0.52, 0.39),
            polygon(0.0, self.obstacle_y, 0.0, 0.10, 0.10))
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'odom'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = -1.0
        goal.target_pose.pose.orientation.z = 1.0
        self.tracking_started = True
        goal_sent_at = rospy.Time.now().to_sec()
        self.client.send_goal(goal)
        deadline = time.time() + (8.0 if self.expect_blocked else 40.0)
        while not rospy.is_shutdown() and time.time() < deadline:
            states = [state for unused_stamp, state in self.state_history]
            if self.expect_blocked and 'BLOCKED' in states:
                rospy.sleep(0.5)
                break
            if (not self.expect_blocked and
                    self.client.wait_for_result(rospy.Duration(0.05))):
                break
            if self.state == 'FAILED' or self.overlap_samples:
                break
            rospy.sleep(0.02)
        state_history_before_terminal = list(self.state_history)
        action_state = self.client.get_state()
        if self.expect_blocked or action_state in (
                GoalStatus.PENDING, GoalStatus.ACTIVE):
            self.client.cancel_goal()
        rospy.sleep(0.2)
        states = [state for unused_stamp, state
                  in state_history_before_terminal]
        blocked_seen = 'BLOCKED' in states
        aligning_seen = 'ALIGNING' in states
        pass_through_seen = 'PASS_THROUGH' in states
        blocked_stamps = [stamp for stamp, state
                          in state_history_before_terminal
                          if state == 'BLOCKED']
        first_blocked = min(blocked_stamps) if blocked_stamps else None
        commands_after_block = [
            (linear, angular) for stamp, unused_state, linear, angular
            in self.command_history
            if first_blocked is not None and stamp >= first_blocked + 0.05]
        maximum_after_block = max(
            [max(abs(linear), abs(angular))
             for linear, angular in commands_after_block] or [0.0])
        commands_after_goal = [
            (linear, angular) for stamp, unused_state, linear, angular
            in self.command_history if stamp >= goal_sent_at]
        maximum_after_goal = max(
            [max(abs(linear), abs(angular))
             for linear, angular in commands_after_goal] or [0.0])
        no_geometric_collision = not self.overlap_samples
        final_pose = self.odom.pose.pose
        final_xy_error = math.hypot(final_pose.position.x + 1.0,
                                    final_pose.position.y)
        final_yaw_error = abs((self.yaw(final_pose) - math.pi + math.pi) %
                              (2.0 * math.pi) - math.pi)
        arrived = (action_state == GoalStatus.SUCCEEDED and
                   final_xy_error <= 0.06 and final_yaw_error <= 0.06)
        if self.expect_blocked:
            behavior_ok = (blocked_seen and maximum_after_goal <= 1.0e-6 and
                           maximum_after_block <= 1.0e-6)
        else:
            behavior_ok = (aligning_seen and pass_through_seen and
                           not blocked_seen and arrived)
        safe = (self.obstacle_in_costmap and not initial_overlap and
                no_geometric_collision and behavior_ok)
        result = {
            'safe': safe,
            'expected_blocked': self.expect_blocked,
            'blocked_seen': blocked_seen,
            'aligning_seen': aligning_seen,
            'pass_through_seen': pass_through_seen,
            'action_state': action_state,
            'arrived': arrived,
            'final_xy_error': final_xy_error,
            'final_yaw_error': final_yaw_error,
            'maximum_command_after_goal': maximum_after_goal,
            'maximum_command_after_block': maximum_after_block,
            'initial_overlap': initial_overlap,
            'alignment_sample_count': self.alignment_samples,
            'overlap_sample_count': len(self.overlap_samples),
            'first_overlap': (self.overlap_samples[0]
                              if self.overlap_samples else None),
            'obstacle_in_costmap': self.obstacle_in_costmap,
            'obstacle_cost': self.obstacle_cost,
            'obstacle': {'center_x': 0.0, 'center_y': self.obstacle_y,
                         'size_x': 0.20, 'size_y': 0.20,
                         'size_z': 1.00},
            'robot_footprint_with_padding': {
                'size_x': 1.04, 'size_y': 0.78},
            'gate_state_at_end': self.state,
        }
        self.write(result)
        rospy.logwarn('[REAR_OBSTACLE_SAFETY] %s', json.dumps(result,
                                                               sort_keys=True))
        return 0 if safe else 2


if __name__ == '__main__':
    rospy.init_node('rear_obstacle_safety_probe')
    raise SystemExit(RearObstacleSafetyProbe().run())
