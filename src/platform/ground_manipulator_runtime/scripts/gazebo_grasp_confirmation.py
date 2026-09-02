#!/usr/bin/env python3
"""Translate Gazebo contact details into backend-neutral grasp state."""

import threading
import time

from gazebo_msgs.msg import ContactsState
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from ground_manipulator_runtime.grasp_confirmation import (
    GraspConfirmationError,
    GraspConfirmationEvaluator,
)


class GazeboGraspConfirmation:
    def __init__(self):
        self._lock = threading.Lock()
        self._evaluator = GraspConfirmationEvaluator(
            rospy.get_param("~contact_max_age", 0.30),
            rospy.get_param("~joint_max_age", 0.50),
            rospy.get_param("~minimum_closed_joint", 0.20))
        self._publisher = rospy.Publisher(
            rospy.get_param(
                "~output_topic", "/ground/gripper/grasp_confirmed"),
            Bool, queue_size=1)
        self._contacts = rospy.Subscriber(
            rospy.get_param("~contact_topic", "/pick_target/contacts"),
            ContactsState, self._contact_callback, queue_size=10)
        self._joints = rospy.Subscriber(
            rospy.get_param("~joint_state_topic", "/ground/joint_states"),
            JointState, self._joint_callback, queue_size=10)
        self._timer = rospy.Timer(rospy.Duration(0.05), self._publish)

    def _contact_callback(self, message):
        pairs = tuple(
            (state.collision1_name, state.collision2_name)
            for state in message.states)
        with self._lock:
            self._evaluator.update_contacts(pairs, time.monotonic())

    def _joint_callback(self, message):
        try:
            with self._lock:
                self._evaluator.update_joint_state(
                    message.name, message.position, time.monotonic())
        except GraspConfirmationError as error:
            rospy.logwarn_throttle(1.0, "invalid gripper state: %s", error)

    def _publish(self, _event):
        with self._lock:
            confirmed = self._evaluator.confirmed(time.monotonic())
        self._publisher.publish(Bool(data=confirmed))


def main():
    rospy.init_node("gazebo_grasp_confirmation")
    GazeboGraspConfirmation()
    rospy.spin()


if __name__ == "__main__":
    main()
