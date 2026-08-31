#!/usr/bin/env python
from __future__ import division, print_function

import argparse
import math

import actionlib
from actionlib_msgs.msg import GoalStatus
import rospy
import tf.transformations as transformations
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def main():
    parser = argparse.ArgumentParser(
        description='Send a human-selected BUNKER approach pose to move_base')
    parser.add_argument('x', type=float)
    parser.add_argument('y', type=float)
    parser.add_argument('yaw_deg', type=float)
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node('send_approach_pose')
    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    if not client.wait_for_server(rospy.Duration(30.0)):
        rospy.logerr('move_base action server unavailable')
        return 2
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'odom'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = args.x
    goal.target_pose.pose.position.y = args.y
    quaternion = transformations.quaternion_from_euler(
        0.0, 0.0, math.radians(args.yaw_deg))
    (goal.target_pose.pose.orientation.x,
     goal.target_pose.pose.orientation.y,
     goal.target_pose.pose.orientation.z,
     goal.target_pose.pose.orientation.w) = quaternion
    client.send_goal(goal)
    rospy.loginfo('Approach pose sent: x=%.3f y=%.3f yaw=%.2f deg',
                  args.x, args.y, args.yaw_deg)
    if not client.wait_for_result(rospy.Duration(120.0)):
        client.cancel_goal()
        rospy.logerr('Navigation timed out')
        return 3
    if client.get_state() != GoalStatus.SUCCEEDED:
        rospy.logerr('Navigation failed: %s', client.get_goal_status_text())
        return 4
    rospy.loginfo('BUNKER reached the approach pose and stopped')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
