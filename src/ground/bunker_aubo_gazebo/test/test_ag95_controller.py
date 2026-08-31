#!/usr/bin/env python
from __future__ import division

import unittest

import actionlib
import rospy
import rostest
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from gazebo_msgs.srv import GetJointProperties
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from urdf_parser_py.urdf import URDF


class Ag95ControllerTest(unittest.TestCase):
    def command(self, client, position):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['left_outer_knuckle_joint']
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = rospy.Duration(2.0)
        goal.trajectory.points = [point]
        client.send_goal(goal)
        self.assertTrue(client.wait_for_result(rospy.Duration(6.0)))
        self.assertEqual(GoalStatus.SUCCEEDED, client.get_state(),
                         str(client.get_result()))

    def joint_positions(self):
        state = rospy.wait_for_message('/joint_states', JointState, timeout=10.0)
        return dict(zip(state.name, state.position))

    def test_urdf_contract(self):
        robot = URDF.from_parameter_server('/robot_description')
        master = robot.joint_map['left_outer_knuckle_joint']
        follower = robot.joint_map['right_outer_knuckle_joint']
        self.assertAlmostEqual(0.0, master.limit.lower)
        self.assertAlmostEqual(0.93, master.limit.upper)
        self.assertEqual('left_outer_knuckle_joint', follower.mimic.joint)
        transmission_joints = [joint.name for transmission in robot.transmissions
                               for joint in transmission.joints]
        self.assertIn('left_outer_knuckle_joint', transmission_joints)

    def test_open_close_and_finger_synchronization(self):
        client = actionlib.SimpleActionClient(
            '/gripper_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction)
        self.assertTrue(client.wait_for_server(rospy.Duration(30.0)))
        self.command(client, 0.0)
        opened = self.joint_positions()
        self.command(client, 0.7)
        closed = self.joint_positions()
        self.assertGreater(closed['left_outer_knuckle_joint'] -
                           opened['left_outer_knuckle_joint'], 0.5)
        rospy.wait_for_service('/gazebo/get_joint_properties', timeout=10.0)
        get_joint = rospy.ServiceProxy('/gazebo/get_joint_properties',
                                       GetJointProperties)
        follower = get_joint('ag95::right_outer_knuckle_joint')
        self.assertTrue(follower.success, follower.status_message)
        self.assertTrue(follower.position)
        self.assertAlmostEqual(closed['left_outer_knuckle_joint'],
                               follower.position[0], delta=0.08)


if __name__ == '__main__':
    rospy.init_node('test_ag95_controller')
    rostest.rosrun('bunker_aubo_gazebo', 'test_ag95_controller', Ag95ControllerTest)
