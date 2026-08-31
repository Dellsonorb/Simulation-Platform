#!/usr/bin/env python
from __future__ import division

import math
import unittest

import actionlib
import rospy
import rostest
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.srv import GetJointProperties, GetModelState
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from trajectory_msgs.msg import JointTrajectoryPoint


class Ag95EffortRuntimeTest(unittest.TestCase):
    PASSIVE = ('right_outer_knuckle_joint',)
    FIXED_HELPERS = (
        'left_finger_joint', 'right_finger_joint',
        'left_inner_knuckle_joint', 'right_inner_knuckle_joint')

    def command(self, client, position, accept_measured=False, duration=3.0):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['left_outer_knuckle_joint']
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = rospy.Duration(duration)
        goal.trajectory.points = [point]
        client.send_goal(goal)
        samples = []
        state = None
        deadline = rospy.Time.now() + rospy.Duration(duration + 5.0)
        while not client.wait_for_result(rospy.Duration(0.05)):
            state = rospy.wait_for_message('/joint_states', JointState,
                                           timeout=2.0)
            positions = dict(zip(state.name, state.position))
            samples.append(positions['left_outer_knuckle_joint'])
            self.assertLess(rospy.Time.now(), deadline)
        if client.get_state() != GoalStatus.SUCCEEDED:
            if state is None:
                state = rospy.wait_for_message('/joint_states', JointState,
                                               timeout=2.0)
            measured = dict(zip(state.name, state.position))[
                'left_outer_knuckle_joint']
            self.assertTrue(accept_measured and
                            abs(measured - position) <= 0.08,
                            'target=%.3f measured=%.6f result=%s' %
                            (position, measured, client.get_result()))
        self.assertTrue(samples)
        self.assertTrue(all(not math.isnan(value) and not math.isinf(value)
                            for value in samples))
        return samples

    def actual_joint(self, service, name):
        result = service('bunker_aubo::' + name)
        self.assertTrue(result.success, result.status_message)
        self.assertEqual(1, len(result.position))
        self.assertFalse(math.isnan(result.position[0]))
        self.assertFalse(math.isinf(result.position[0]))
        return result.position[0]

    def test_open_close_tracks_without_teleport_or_model_instability(self):
        client = actionlib.SimpleActionClient(
            '/gripper_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction)
        self.assertTrue(client.wait_for_server(rospy.Duration(40.0)))
        rospy.wait_for_service('/gazebo/unpause_physics', timeout=30.0)
        rospy.ServiceProxy('/gazebo/unpause_physics', Empty)()
        rospy.wait_for_service('/gazebo/get_joint_properties', timeout=30.0)
        rospy.wait_for_service('/gazebo/get_model_state', timeout=30.0)
        get_joint = rospy.ServiceProxy('/gazebo/get_joint_properties',
                                       GetJointProperties)
        get_model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        def pad_distance():
            states = rospy.wait_for_message('/gazebo/link_states', LinkStates,
                                            timeout=10.0)
            poses = dict(zip(states.name, states.pose))
            try:
                left_name = next(name for name in poses
                                 if name.endswith('left_finger_pad'))
                right_name = next(name for name in poses
                                  if name.endswith('right_finger_pad'))
            except StopIteration:
                self.fail('finger-pad links absent from %s' % sorted(poses))
            left = poses[left_name].position
            right = poses[right_name].position
            dx = left.x - right.x
            dy = left.y - right.y
            dz = left.z - right.z
            return math.sqrt(dx * dx + dy * dy + dz * dz)

        self.command(client, 0.0, accept_measured=True)
        master_open = self.actual_joint(get_joint, 'left_outer_knuckle_joint')
        passive_open = dict((name, self.actual_joint(get_joint, name))
                            for name in self.PASSIVE)
        for name, position in passive_open.items():
            self.assertAlmostEqual(
                master_open, position, delta=0.10,
                msg='open %s master=%.6f passive=%s' %
                    (name, master_open, passive_open))
        open_pad_distance = pad_distance()
        closed_history = self.command(
            client, 0.70, accept_measured=True, duration=8.0)
        # FJT duration and intermediate samples prove controller motion rather
        # than a SetPosition/teleport jump.
        self.assertGreater(len(closed_history), 5)
        self.assertGreater(max(closed_history) - min(closed_history), 0.50)
        rospy.sleep(1.0)
        master = self.actual_joint(get_joint, 'left_outer_knuckle_joint')
        self.assertAlmostEqual(0.70, master, delta=0.08)
        passive_closed = dict((name, self.actual_joint(get_joint, name))
                              for name in self.PASSIVE)
        for name, position in passive_closed.items():
            self.assertAlmostEqual(
                master, position, delta=0.10,
                msg='%s master=%.6f passive=%s' %
                    (name, master, passive_closed))
        closed_pad_distance = pad_distance()
        self.assertLess(
            closed_pad_distance, open_pad_distance - 0.03,
            'pad centers did not close: open=%.6f closed=%.6f' %
            (open_pad_distance, closed_pad_distance))

        self.command(client, 0.0, accept_measured=True)
        master = self.actual_joint(get_joint, 'left_outer_knuckle_joint')
        self.assertAlmostEqual(0.0, master, delta=0.06)
        for name in self.PASSIVE:
            self.assertAlmostEqual(master, self.actual_joint(get_joint, name),
                                   delta=0.10, msg=name)
        model = get_model('bunker_aubo', 'world')
        self.assertTrue(model.success, model.status_message)
        values = (model.pose.position.x, model.pose.position.y,
                  model.pose.position.z, model.pose.orientation.w)
        self.assertTrue(all(not math.isnan(value) and not math.isinf(value)
                            for value in values))


if __name__ == '__main__':
    rospy.init_node('test_ag95_effort_runtime')
    rostest.rosrun('bunker_aubo_gazebo', 'test_ag95_effort_runtime',
                   Ag95EffortRuntimeTest)
