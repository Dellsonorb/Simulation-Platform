#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

from bunker_sim_runtime.velocity_guard import admit_components


class VelocityGuard:
    def __init__(self):
        self._publisher = rospy.Publisher(
            "cmd_vel_safe", Twist, queue_size=1)
        self._subscriber = rospy.Subscriber(
            "cmd_vel", Twist, self._callback, queue_size=1)

    def _callback(self, message):
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


def main():
    rospy.init_node("velocity_guard")
    VelocityGuard()
    rospy.spin()


if __name__ == "__main__":
    main()
