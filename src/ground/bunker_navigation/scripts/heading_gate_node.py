#!/usr/bin/env python
from __future__ import division

import math
import threading

import rospy
import tf
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String

from bunker_navigation.costmap_sweep import (CostmapGrid,
                                             check_rotation_sweep,
                                             pad_footprint)
from bunker_navigation.heading_gate import GoalLifecycle, HeadingGate


class HeadingGateNode(object):
    def __init__(self):
        degree = math.pi / 180.0
        self.gate = HeadingGate(
            rospy.get_param('~entry_bearing_deg', 100.0) * degree,
            rospy.get_param('~release_bearing_deg', 30.0) * degree,
            rospy.get_param('~minimum_distance', 0.5),
            rospy.get_param('~angular_gain', 1.5),
            rospy.get_param('~minimum_angular_speed', 0.25),
            rospy.get_param('~maximum_angular_speed', 0.8),
            rospy.get_param('~alignment_timeout', 15.0),
            rospy.get_param('~command_watchdog', 0.5))
        self.lock = threading.RLock()
        self.lifecycle = GoalLifecycle()
        self.listener = tf.TransformListener()
        self.robot_pose = None
        self.raw_command = (0.0, 0.0)
        self.raw_stamp = float('-inf')
        footprint = rospy.get_param(
            '~footprint',
            [[-0.50, -0.37], [-0.50, 0.37],
             [0.50, 0.37], [0.50, -0.37]])
        self.footprint = pad_footprint(
            [(float(point[0]), float(point[1])) for point in footprint],
            rospy.get_param('~footprint_padding', 0.02))
        self.costmap = None
        self.costmap_received_at = float('-inf')
        self.costmap_frame = rospy.get_param('~costmap_frame', 'odom')
        self.costmap_max_age = rospy.get_param('~costmap_max_age', 0.75)
        self.sweep_angular_step = rospy.get_param(
            '~sweep_angular_step', 0.025)
        self.lethal_cost_threshold = rospy.get_param(
            '~lethal_cost_threshold', 100)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.state_publisher = rospy.Publisher(
            '~state', String, queue_size=1, latch=True)
        rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber(
            rospy.get_param(
                '~costmap_topic', '/move_base/local_costmap/costmap'),
            OccupancyGrid, self.costmap_callback, queue_size=1)
        rospy.Subscriber('/navigation/raw_cmd_vel', Twist,
                         self.raw_callback, queue_size=1)
        rospy.Subscriber('/move_base/goal', MoveBaseActionGoal,
                         self.goal_callback, queue_size=1)
        rospy.Subscriber('/move_base/cancel', GoalID,
                         self.cancel_callback, queue_size=1)
        rospy.Subscriber('/move_base/result', MoveBaseActionResult,
                         self.result_callback, queue_size=1)
        rospy.Timer(rospy.Duration(0.05), self.tick)

    @staticmethod
    def yaw(orientation):
        return tf.transformations.euler_from_quaternion(
            (orientation.x, orientation.y,
             orientation.z, orientation.w))[2]

    def odom_callback(self, message):
        with self.lock:
            pose = message.pose.pose
            self.robot_pose = (pose.position.x, pose.position.y,
                               self.yaw(pose.orientation))

    def raw_callback(self, message):
        with self.lock:
            self.raw_command = (message.linear.x, message.angular.z)
            self.raw_stamp = rospy.Time.now().to_sec()

    def costmap_callback(self, message):
        received_at = rospy.Time.now().to_sec()
        try:
            if message.header.frame_id != self.costmap_frame:
                raise ValueError(
                    'costmap frame %s does not match %s' %
                    (message.header.frame_id, self.costmap_frame))
            if abs(self.yaw(message.info.origin.orientation)) > 1.0e-6:
                raise ValueError('rotated OccupancyGrid origins are unsupported')
            grid = CostmapGrid(
                message.info.width, message.info.height,
                message.info.resolution,
                message.info.origin.position.x,
                message.info.origin.position.y,
                message.data)
        except ValueError as error:
            rospy.logerr_throttle(1.0, 'heading gate invalid costmap: %s', error)
            grid = None
        with self.lock:
            self.costmap = grid
            self.costmap_received_at = received_at

    def goal_callback(self, message):
        goal_id = message.goal_id.id
        goal_stamp = message.goal_id.stamp.to_sec()
        with self.lock:
            self.lifecycle.start(goal_id, goal_stamp)
            self.gate.clear()
            self._invalidate_raw_command()
            state = self.gate.state
        self._publish_stop(state)
        try:
            target = message.goal.target_pose
            if target.header.frame_id in ('', 'odom'):
                goal = target
            else:
                goal = self.listener.transformPose('odom', target)
            with self.lock:
                if self.lifecycle.active_goal_id != goal_id:
                    return
                self._invalidate_raw_command()
                if self.robot_pose is None:
                    self.gate.fail()
                else:
                    self.gate.set_goal(
                        (goal.pose.position.x, goal.pose.position.y),
                        self.robot_pose, rospy.Time.now().to_sec())
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            with self.lock:
                if self.lifecycle.active_goal_id == goal_id:
                    self.gate.fail()

    def cancel_callback(self, message):
        with self.lock:
            matched = self.lifecycle.cancel(
                message.id, message.stamp.to_sec())
            if not matched:
                return
            self.gate.clear()
            self._invalidate_raw_command()
            state = self.gate.state
        self._publish_stop(state)

    def result_callback(self, message):
        with self.lock:
            if not self.lifecycle.finish(message.status.goal_id.id):
                return
            self.gate.clear()
            self._invalidate_raw_command()
            state = self.gate.state
        self._publish_stop(state)

    def _invalidate_raw_command(self):
        self.raw_command = (0.0, 0.0)
        self.raw_stamp = float('-inf')

    def _publish_stop(self, state):
        self.publisher.publish(Twist())
        self.state_publisher.publish(String(data=state))

    def tick(self, unused_event):
        output = Twist()
        blocked_reason = None
        with self.lock:
            if self.robot_pose is not None:
                now = rospy.Time.now().to_sec()
                if self.gate.state == HeadingGate.ALIGNING:
                    if self.costmap is None:
                        blocked_reason = 'COSTMAP_UNAVAILABLE'
                    elif (now - self.costmap_received_at < 0.0 or
                          now - self.costmap_received_at >
                          self.costmap_max_age):
                        blocked_reason = 'COSTMAP_STALE'
                    else:
                        sweep = check_rotation_sweep(
                            self.costmap, self.footprint,
                            self.robot_pose, self.gate.goal,
                            self.gate.release_bearing,
                            self.sweep_angular_step,
                            self.lethal_cost_threshold)
                        if not sweep.clear:
                            blocked_reason = '%s at yaw %.3f' % (
                                sweep.reason, sweep.collision_yaw)
                    if blocked_reason is not None:
                        self.gate.block()
                linear, angular = self.gate.command(
                    self.robot_pose, self.raw_command,
                    now, self.raw_stamp)
                output.linear.x = linear
                output.angular.z = angular
            state = self.gate.state
        if blocked_reason is not None:
            rospy.logerr('heading gate BLOCKED: %s; publishing zero velocity',
                         blocked_reason)
        self.publisher.publish(output)
        self.state_publisher.publish(String(data=state))


if __name__ == '__main__':
    rospy.init_node('heading_gate')
    HeadingGateNode()
    rospy.spin()
