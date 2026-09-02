#!/usr/bin/env python3
import math
import time

import rospy
from geometry_msgs.msg import Twist

from bunker_sim_runtime.velocity_guard import admit_components, command_is_fresh


class VelocityGuard:
    def __init__(self):
        self._timeout = float(rospy.get_param("~command_timeout", 0.5))
        if not math.isfinite(self._timeout) or self._timeout <= 0.0:
            raise ValueError("~command_timeout must be positive")
        self._last_input = None
        self._publisher = rospy.Publisher(
            "cmd_vel", Twist, queue_size=1)
        self._subscriber = rospy.Subscriber(
            "nav_cmd_vel", Twist, self._callback, queue_size=1)
        self._timer = rospy.Timer(
            rospy.Duration(min(0.1, self._timeout / 2.0)), self._watchdog)

    def _callback(self, message):
        self._last_input = time.monotonic()
        admitted, reason = admit_components(
            message.linear.x, message.linear.y, message.linear.z,
            message.angular.x, message.angular.y, message.angular.z)
        output = Twist()
        output.linear.x, output.linear.y, output.linear.z = admitted[:3]
        output.angular.x, output.angular.y, output.angular.z = admitted[3:]
        if reason in ("nonfinite", "nonplanar"):
            rospy.logerr_throttle(
                1.0, "velocity guard rejected %s input" % reason)
        elif reason == "clamped":
            rospy.logwarn_throttle(1.0, "velocity guard clamped input")
        self._publisher.publish(output)

    def _watchdog(self, _event):
        if not command_is_fresh(
                self._last_input, time.monotonic(), self._timeout):
            self._publisher.publish(Twist())


def main():
    rospy.init_node("velocity_guard")
    VelocityGuard()
    rospy.spin()


if __name__ == "__main__":
    main()
