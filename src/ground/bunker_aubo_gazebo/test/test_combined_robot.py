#!/usr/bin/env python
from __future__ import division

import math
import time
import unittest

import actionlib
import rospy
import rostest
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from gazebo_msgs.srv import GetModelState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


class CombinedRobotTest(unittest.TestCase):
    def client(self, name):
        client = actionlib.SimpleActionClient(
            name + '/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.assertTrue(client.wait_for_server(rospy.Duration(40.0)))
        return client

    def command(self, client, joints, positions, duration=2.0):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = joints
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(duration)
        goal.trajectory.points = [point]
        client.send_goal(goal)
        self.assertTrue(client.wait_for_result(rospy.Duration(duration + 5.0)))
        self.assertEqual(GoalStatus.SUCCEEDED, client.get_state(),
                         str(client.get_result()))

    def test_model_stability_and_real_controllers(self):
        rospy.wait_for_service('/gazebo/get_model_state', timeout=30.0)
        get_model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        deadline = time.time() + 30.0
        state = None
        while time.time() < deadline:
            state = get_model('bunker_aubo', 'world')
            if state.success:
                break
            time.sleep(0.1)
        self.assertTrue(state.success, state.status_message)
        values = [state.pose.position.x, state.pose.position.y,
                  state.pose.position.z, state.pose.orientation.w]
        self.assertTrue(all(not math.isnan(value) and not math.isinf(value)
                            for value in values))

        joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                  'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        rospy.sleep(1.0)
        before_msg = rospy.wait_for_message('/joint_states', JointState, timeout=20.0)
        before = dict(zip(before_msg.name, before_msg.position))
        target = [before[name] for name in joints]
        target[2] = max(-2.5, min(2.5, target[2] - 0.15))
        self.command(self.client('/arm_controller'), joints, target, 3.0)
        after_msg = rospy.wait_for_message('/joint_states', JointState, timeout=10.0)
        after = dict(zip(after_msg.name, after_msg.position))
        self.assertGreater(abs(after['elbow_joint'] - before['elbow_joint']), 0.1)

        gripper = self.client('/gripper_controller')
        self.command(gripper, ['left_outer_knuckle_joint'], [0.0])
        self.command(gripper, ['left_outer_knuckle_joint'], [0.7])
        closed = rospy.wait_for_message('/joint_states', JointState, timeout=10.0)
        positions = dict(zip(closed.name, closed.position))
        self.assertAlmostEqual(0.7, positions['left_outer_knuckle_joint'], delta=0.08)


if __name__ == '__main__':
    rospy.init_node('test_combined_robot')
    rostest.rosrun('bunker_aubo_gazebo', 'test_combined_robot', CombinedRobotTest)
