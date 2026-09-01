#!/usr/bin/env python
from __future__ import division

import sys
import unittest

import moveit_commander
import rospy
import rostest
from std_msgs.msg import Bool


class MoveItExecuteTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        moveit_commander.roscpp_initialize(sys.argv)

    def test_rrtconnect_plan_and_real_execute(self):
        ready = rospy.wait_for_message('/ground/runtime_ready', Bool,
                                       timeout=60.0)
        self.assertTrue(ready.data)

        group = moveit_commander.MoveGroupCommander('manipulator', wait_for_servers=60.0)
        group.set_planner_id('RRTConnectkConfigDefault')
        group.set_planning_time(10.0)
        group.set_start_state_to_current_state()
        before = group.get_current_joint_values()
        active_joints = group.get_active_joints()
        elbow_index = active_joints.index('elbow_joint')
        target = list(before)
        target[elbow_index] = max(
            -2.5, min(2.5, target[elbow_index] - 0.2))
        group.set_joint_value_target(target)
        plan = group.plan()
        trajectory = plan[1] if isinstance(plan, tuple) else plan
        self.assertTrue(trajectory.joint_trajectory.points)
        self.assertTrue(group.execute(trajectory, wait=True))
        group.stop()
        after = group.get_current_joint_values()
        self.assertGreater(
            abs(after[elbow_index] - before[elbow_index]), 0.12)
        self.assertAlmostEqual(
            target[elbow_index], after[elbow_index], delta=0.08)


if __name__ == '__main__':
    rospy.init_node('test_moveit_execute')
    rostest.rosrun('bunker_aubo_moveit_config', 'test_moveit_execute', MoveItExecuteTest)
