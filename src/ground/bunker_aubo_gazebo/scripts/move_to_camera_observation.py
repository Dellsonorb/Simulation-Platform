#!/usr/bin/env python
from __future__ import print_function

import sys

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


def actual_positions(joint_names, timeout):
    message = rospy.wait_for_message('/joint_states', JointState, timeout=timeout)
    positions = dict(zip(message.name, message.position))
    return [positions[name] for name in joint_names]


def main():
    rospy.init_node('move_to_camera_observation')
    joint_names = rospy.get_param('~joint_names')
    positions = rospy.get_param('~positions')
    duration = float(rospy.get_param('~duration', 5.0))
    tolerance = float(rospy.get_param('~goal_tolerance', 0.08))
    if len(joint_names) != 6 or len(positions) != 6:
        rospy.logerr('Observation pose must contain exactly six AUBO joints')
        return 2

    client = actionlib.SimpleActionClient(
        '/arm_controller/follow_joint_trajectory',
        FollowJointTrajectoryAction)
    rospy.loginfo('Waiting for the Gazebo AUBO trajectory controller')
    if not client.wait_for_server(rospy.Duration(30.0)):
        rospy.logerr('AUBO trajectory controller is unavailable')
        return 3

    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = joint_names
    point = JointTrajectoryPoint()
    point.positions = positions
    point.time_from_start = rospy.Duration(duration)
    goal.trajectory.points = [point]
    goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.2)
    client.send_goal(goal)
    if not client.wait_for_result(rospy.Duration(duration + 15.0)):
        client.cancel_goal()
        rospy.logerr('Timed out moving to the D435 observation pose')
        return 4
    if client.get_state() != GoalStatus.SUCCEEDED:
        rospy.logerr('Observation trajectory failed: %s', client.get_goal_status_text())
        return 5

    measured = actual_positions(joint_names, 5.0)
    maximum_error = max(abs(actual - target)
                        for actual, target in zip(measured, positions))
    if maximum_error > tolerance:
        rospy.logerr('Observation joint error %.4f exceeds %.4f',
                     maximum_error, tolerance)
        return 6
    rospy.loginfo('D435 observation pose reached; maximum joint error %.4f rad',
                  maximum_error)
    return 0


if __name__ == '__main__':
    sys.exit(main())
