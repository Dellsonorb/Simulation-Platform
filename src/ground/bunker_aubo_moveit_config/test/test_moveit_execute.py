#!/usr/bin/env python
from __future__ import division

import sys
import unittest

import moveit_commander
import rospy
import rostest


class MoveItExecuteTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        moveit_commander.roscpp_initialize(sys.argv)

    def test_rrtconnect_plan_and_real_execute(self):
        group = moveit_commander.MoveGroupCommander('manipulator', wait_for_servers=60.0)
        group.set_planner_id('RRTConnectkConfigDefault')
        group.set_planning_time(10.0)
        group.set_start_state_to_current_state()
        before = group.get_current_joint_values()
        target = list(before)
        target[2] = max(-2.5, min(2.5, target[2] - 0.2))
        group.set_joint_value_target(target)
        plan = group.plan()
        trajectory = plan[1] if isinstance(plan, tuple) else plan
        self.assertTrue(trajectory.joint_trajectory.points)
        self.assertTrue(group.execute(trajectory, wait=True))
        group.stop()
        after = group.get_current_joint_values()
        self.assertGreater(abs(after[2] - before[2]), 0.12)
        self.assertAlmostEqual(target[2], after[2], delta=0.08)


if __name__ == '__main__':
    rospy.init_node('test_moveit_execute')
    rostest.rosrun('bunker_aubo_moveit_config', 'test_moveit_execute', MoveItExecuteTest)
