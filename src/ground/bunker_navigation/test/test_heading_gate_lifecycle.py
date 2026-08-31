#!/usr/bin/env python
from __future__ import division

import threading
import time
import unittest

import rospy
import rostest
from actionlib_msgs.msg import GoalID, GoalStatus
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String


class HeadingGateLifecycleRuntimeTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.latest_command = None
        self.latest_state = None
        self.command_count = 0
        self.commands = []
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.raw_pub = rospy.Publisher('/navigation/raw_cmd_vel', Twist,
                                       queue_size=1)
        self.goal_pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal,
                                        queue_size=1)
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID,
                                          queue_size=1)
        self.result_pub = rospy.Publisher('/move_base/result',
                                          MoveBaseActionResult, queue_size=1)
        self.costmap_pub = rospy.Publisher(
            '/move_base/local_costmap/costmap', OccupancyGrid,
            queue_size=1, latch=True)
        rospy.Subscriber('/cmd_vel', Twist, self.command_callback,
                         queue_size=10)
        rospy.Subscriber('/heading_gate/state', String, self.state_callback,
                         queue_size=10)
        deadline = time.time() + 8.0
        while (not rospy.is_shutdown() and time.time() < deadline and
               (self.goal_pub.get_num_connections() == 0 or
                self.result_pub.get_num_connections() == 0 or
                self.cancel_pub.get_num_connections() == 0)):
            rospy.sleep(0.05)
        self.assertGreater(self.goal_pub.get_num_connections(), 0)
        cancel_all = GoalID()
        self.cancel_pub.publish(cancel_all)
        self.publish_odom()
        self.publish_costmap()
        self.wait_state('IDLE')

    def command_callback(self, message):
        with self.lock:
            self.latest_command = (message.linear.x, message.angular.z)
            self.command_count += 1
            self.commands.append(self.latest_command)

    def state_callback(self, message):
        with self.lock:
            self.latest_state = message.data

    def publish_odom(self):
        message = Odometry()
        message.header.frame_id = 'odom'
        message.child_frame_id = 'base_link'
        message.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(message)
        rospy.sleep(0.1)

    def publish_raw(self, linear=-0.2, angular=0.1):
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        self.raw_pub.publish(message)

    def publish_costmap(self, obstacle=None):
        message = OccupancyGrid()
        message.header.frame_id = 'odom'
        message.header.stamp = rospy.Time.now()
        message.info.width = 200
        message.info.height = 200
        message.info.resolution = 0.02
        message.info.origin.position.x = -2.0
        message.info.origin.position.y = -2.0
        message.info.origin.orientation.w = 1.0
        message.data = [0] * (message.info.width * message.info.height)
        if obstacle is not None:
            mx = int((obstacle[0] + 2.0) / message.info.resolution)
            my = int((obstacle[1] + 2.0) / message.info.resolution)
            message.data[my * message.info.width + mx] = 100
        self.costmap_pub.publish(message)
        rospy.sleep(0.1)

    def publish_goal(self, goal_id, x):
        message = MoveBaseActionGoal()
        message.goal_id.id = goal_id
        message.goal_id.stamp = rospy.Time.now()
        message.goal.target_pose.header.frame_id = 'odom'
        message.goal.target_pose.pose.position.x = x
        message.goal.target_pose.pose.orientation.w = 1.0
        self.goal_pub.publish(message)

    def publish_result(self, goal_id, status):
        message = MoveBaseActionResult()
        message.status.goal_id.id = goal_id
        message.status.status = status
        self.result_pub.publish(message)

    def wait_state(self, expected, timeout=3.0):
        deadline = time.time() + timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            with self.lock:
                if self.latest_state == expected:
                    return
            rospy.sleep(0.02)
        self.fail('state did not become {} (latest={})'.format(
            expected, self.latest_state))

    def assert_stopped(self, timeout=1.0):
        with self.lock:
            after_count = self.command_count
        self.assert_new_stop(after_count, timeout)

    def assert_new_stop(self, after_count, timeout=1.0):
        deadline = time.time() + timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            with self.lock:
                command = self.latest_command
                command_count = self.command_count
            if command_count > after_count and command == (0.0, 0.0):
                return
            rospy.sleep(0.01)
        self.fail('new zero command not observed (latest={}, count={})'.format(
            command, command_count))

    def wait_alignment_command(self, timeout=1.0):
        deadline = time.time() + timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            with self.lock:
                command = self.latest_command
            if (command is not None and command[0] == 0.0 and
                    abs(command[1]) > 0.0):
                return
            rospy.sleep(0.01)
        self.fail('alignment rotation command not observed')

    def activate_rear_goal(self, goal_id):
        self.publish_goal(goal_id, -1.0)
        self.wait_state('ALIGNING')
        self.publish_raw()
        self.wait_alignment_command()

    def test_new_goal_cancel_preempt_abort_and_success_reset_safely(self):
        self.activate_rear_goal('old')
        with self.lock:
            marker = self.command_count
        self.publish_goal('replacement', 1.0)
        self.assert_new_stop(marker)
        self.wait_state('PASS_THROUGH')

        cancel = GoalID()
        cancel.id = 'replacement'
        with self.lock:
            marker = self.command_count
        self.cancel_pub.publish(cancel)
        self.wait_state('IDLE')
        self.assert_new_stop(marker)

        for index, status in enumerate((GoalStatus.PREEMPTED,
                                        GoalStatus.ABORTED,
                                        GoalStatus.SUCCEEDED)):
            goal_id = 'terminal-{}'.format(index)
            self.activate_rear_goal(goal_id)
            with self.lock:
                marker = self.command_count
            self.publish_result(goal_id, status)
            self.wait_state('IDLE')
            self.assert_new_stop(marker)

        self.activate_rear_goal('stale-old')
        self.publish_goal('current', -1.0)
        self.publish_result('stale-old', GoalStatus.PREEMPTED)
        self.publish_raw()
        self.wait_state('ALIGNING')

    def test_near_obstacle_latches_blocked_and_never_rotates(self):
        self.publish_costmap((0.0, 0.58))
        with self.lock:
            marker = self.command_count
            command_marker = len(self.commands)
        self.publish_goal('blocked', -1.0)
        self.publish_raw()
        self.wait_state('BLOCKED')
        self.assert_new_stop(marker)
        rospy.sleep(0.2)
        with self.lock:
            self.assertEqual(self.latest_command, (0.0, 0.0))
            self.assertTrue(all(command == (0.0, 0.0)
                                for command in self.commands[command_marker:]))

    def test_obstacle_appearing_during_alignment_is_checked_and_blocks(self):
        self.publish_costmap()
        self.activate_rear_goal('dynamic-obstacle')
        with self.lock:
            marker = self.command_count
        self.publish_costmap((0.0, 0.58))
        self.wait_state('BLOCKED')
        self.assert_new_stop(marker)


if __name__ == '__main__':
    rospy.init_node('test_heading_gate_lifecycle')
    rostest.rosrun('bunker_navigation', 'heading_gate_lifecycle',
                   HeadingGateLifecycleRuntimeTest)
