#!/usr/bin/env python
from __future__ import division

import time
import unittest

import rospy
import rostest
from gazebo_msgs.srv import GetModelState


class FullPickTest(unittest.TestCase):
    def test_brick_is_lifted(self):
        rospy.wait_for_service('/gazebo/get_model_state', timeout=30.0)
        get_model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        deadline = time.time() + 90.0
        while time.time() < deadline and not rospy.is_shutdown():
            if rospy.get_param('/brick_pick_demo/success', False):
                state = get_model('brick', 'world')
                self.assertTrue(state.success, state.status_message)
                self.assertGreater(state.pose.position.z, 0.14)
                return
            time.sleep(0.2)
        self.fail('brick pick did not report SUCCESS within 90 seconds')


if __name__ == '__main__':
    rospy.init_node('test_full_pick')
    rostest.rosrun('brick_pick_demo', 'test_full_pick', FullPickTest)
