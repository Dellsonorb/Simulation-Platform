#!/usr/bin/env python
import unittest
import rospy
import rostest
from srdfdom.srdf import SRDF


class MoveItConfigTest(unittest.TestCase):
    def test_semantic_groups_and_controller_contract(self):
        robot_description = rospy.get_param('/robot_description')
        self.assertIn('ground/aubo_i5_base_link', robot_description)
        self.assertIn('ground/gripper_tcp_link', robot_description)

        srdf = SRDF.from_parameter_server('/robot_description_semantic')
        groups = {group.name: group for group in srdf.groups}
        self.assertIn('manipulator', groups)
        self.assertIn('gripper', groups)
        self.assertTrue(srdf.end_effectors)
        self.assertEqual('ag95', srdf.end_effectors[0].name)
        self.assertEqual('RRTConnectkConfigDefault',
                         rospy.get_param(
                             '/move_group/manipulator/default_planner_config'))
        controllers = rospy.get_param('/move_group/controller_list')
        self.assertEqual({'/ground/arm_controller',
                          '/ground/gripper_controller'},
                         {item['name'] for item in controllers})
        self.assertTrue(all(item.get('default') for item in controllers))
        self.assertFalse(any('fake' in item['name'] for item in controllers))


if __name__ == '__main__':
    rospy.init_node('test_moveit_config')
    rostest.rosrun('bunker_aubo_moveit_config', 'test_moveit_config', MoveItConfigTest)
