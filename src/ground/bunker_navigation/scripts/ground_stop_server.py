#!/usr/bin/env python3
"""Expose the common ground stop operation as a small ROS service."""

import actionlib
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction
import rospy
from std_srvs.srv import Trigger, TriggerResponse

from bunker_navigation.stop import stop_robot


class GroundStopServer:
    def __init__(self):
        self._move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self._publisher = rospy.Publisher("nav_cmd_vel", Twist, queue_size=1)
        self._service = rospy.Service(
            "runtime/stop", Trigger, self._handle_stop)

    def _handle_stop(self, _request):
        result = stop_robot(self._move_base, self._publisher, Twist)
        return TriggerResponse(**result)


def main():
    rospy.init_node("ground_stop_server")
    GroundStopServer()
    rospy.spin()


if __name__ == "__main__":
    main()
