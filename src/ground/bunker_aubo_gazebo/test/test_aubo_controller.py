#!/usr/bin/env python
from __future__ import division

import unittest

import actionlib
import rospy
import rostest
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from controller_manager_msgs.srv import ListControllers
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


ARM_JOINTS = {
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
}


class AuboControllerTest(unittest.TestCase):
    def test_real_trajectory_controller_contract(self):
        client = actionlib.SimpleActionClient(
            '/arm_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction)
        self.assertTrue(client.wait_for_server(rospy.Duration(30.0)))
        rospy.wait_for_service('/controller_manager/list_controllers', timeout=30.0)
        controllers = rospy.ServiceProxy(
            '/controller_manager/list_controllers', ListControllers)()
        states = {item.name: item.state for item in controllers.controller}
        self.assertEqual('running', states.get('arm_controller'))
        self.assertEqual('running', states.get('joint_state_controller'))

        rospy.sleep(2.0)
        before = rospy.wait_for_message('/joint_states', JointState, timeout=10.0)
        before_positions = dict(zip(before.name, before.position))
        ordered_joints = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        target = [before_positions[joint] for joint in ordered_joints]
        target[2] = max(-2.5, min(2.5, target[2] - 0.2))
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ordered_joints
        point = JointTrajectoryPoint()
        point.positions = target
        point.time_from_start = rospy.Duration(3.0)
        goal.trajectory.points = [point]
        client.send_goal(goal)
        self.assertTrue(client.wait_for_result(rospy.Duration(8.0)))
        after = rospy.wait_for_message('/joint_states', JointState, timeout=10.0)
        after_positions = dict(zip(after.name, after.position))
        self.assertEqual(GoalStatus.SUCCEEDED, client.get_state(),
                         '{} before={} target={} after={}'.format(
                             client.get_result(), before_positions, target,
                             after_positions))
        self.assertGreater(abs(after_positions['elbow_joint'] -
                               before_positions['elbow_joint']), 0.1)
        for joint, expected in zip(ordered_joints, target):
            self.assertAlmostEqual(expected, after_positions[joint], delta=0.05)

    def test_all_six_arm_joints_are_published(self):
        message = rospy.wait_for_message('/joint_states', JointState, timeout=30.0)
        self.assertTrue(ARM_JOINTS.issubset(set(message.name)))


if __name__ == '__main__':
    rospy.init_node('test_aubo_controller')
    rostest.rosrun('bunker_aubo_gazebo', 'test_aubo_controller', AuboControllerTest)
