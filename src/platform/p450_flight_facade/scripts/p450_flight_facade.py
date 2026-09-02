#!/usr/bin/env python3
"""Expose common flight actions by translating them to Prometheus messages."""

import math
import threading
import time

import actionlib
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from prometheus_msgs.msg import UAVCommand, UAVControlState, UAVSetup, UAVState
from robot_runtime_interfaces.msg import (
    FlightCommandAction,
    FlightCommandFeedback,
    FlightCommandResult,
)
import rospy
from tf2_geometry_msgs import do_transform_pose
import tf2_ros

from p450_flight_facade.translation import (
    FLY_TO,
    HOVER,
    LAND,
    TAKEOFF,
    TranslationError,
    backend_healthy,
    position_complete,
    translation_for,
)


class PrometheusFlightFacade:
    """A procedural adapter; Prometheus remains the flight authority."""

    def __init__(self):
        self._lock = threading.RLock()
        self._state = None
        self._control = None
        self._odom = None
        self._state_received = None
        self._control_received = None
        self._odom_received = None
        self._command_id = 0

        self._command_topic = rospy.get_param(
            "~command_topic", "/uav1/prometheus/command")
        self._setup_topic = rospy.get_param(
            "~setup_topic", "/uav1/prometheus/setup")
        self._state_topic = rospy.get_param(
            "~state_topic", "/uav1/prometheus/state")
        self._control_topic = rospy.get_param(
            "~control_state_topic", "/uav1/prometheus/control_state")
        self._odom_topic = rospy.get_param(
            "~odom_topic", "/uav1/prometheus/odom")
        self._action_name = rospy.get_param(
            "~action_name", "/uav1/runtime/flight")
        self._odom_frame = rospy.get_param("~odom_frame", "uav1/odom")
        self._publish_rate = self._positive("publish_rate", 10.0)
        self._backend_max_age = self._positive("backend_max_age", 0.75)
        self._position_tolerance = self._positive(
            "position_tolerance", 0.15)
        self._settle_speed = self._nonnegative("settle_speed", 0.15)
        self._takeoff_height_param = rospy.get_param(
            "~takeoff_height_param",
            "/uav_control_main_1/control/Takeoff_height")

        self._command_pub = rospy.Publisher(
            self._command_topic, UAVCommand, queue_size=10)
        self._setup_pub = rospy.Publisher(
            self._setup_topic, UAVSetup, queue_size=10)
        rospy.Subscriber(
            self._state_topic, UAVState, self._state_callback, queue_size=1)
        rospy.Subscriber(
            self._control_topic, UAVControlState,
            self._control_callback, queue_size=1)
        rospy.Subscriber(
            self._odom_topic, Odometry, self._odom_callback, queue_size=1)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._server = actionlib.SimpleActionServer(
            self._action_name, FlightCommandAction,
            execute_cb=self._execute, auto_start=False)
        self._server.start()

    @staticmethod
    def _positive(name, default):
        value = float(rospy.get_param("~" + name, default))
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError("~%s must be positive" % name)
        return value

    @staticmethod
    def _nonnegative(name, default):
        value = float(rospy.get_param("~" + name, default))
        if not math.isfinite(value) or value < 0.0:
            raise ValueError("~%s must be nonnegative" % name)
        return value

    def _state_callback(self, message):
        with self._lock:
            self._state = message
            self._state_received = time.monotonic()

    def _control_callback(self, message):
        with self._lock:
            self._control = message
            self._control_received = time.monotonic()

    def _odom_callback(self, message):
        with self._lock:
            self._odom = message
            self._odom_received = time.monotonic()

    def _snapshot(self):
        with self._lock:
            return (
                self._state,
                self._control,
                self._odom,
                self._state_received,
                self._control_received,
                self._odom_received,
            )

    def _health(self, snapshot=None):
        snapshot = snapshot or self._snapshot()
        state, control, odom, state_at, control_at, odom_at = snapshot
        if any(value is None for value in (
                state, control, odom, state_at, control_at, odom_at)):
            return False
        now = time.monotonic()
        return bool(
            now - odom_at <= self._backend_max_age
            and backend_healthy(
                state.connected, state.odom_valid, state.armed,
                control.failsafe, now - state_at, now - control_at,
                self._backend_max_age))

    def _publish_setup(self, operation, value):
        message = UAVSetup()
        message.header.stamp = rospy.Time.now()
        if operation == "ARMING":
            message.cmd = UAVSetup.ARMING
            message.arming = bool(value)
        elif operation == "SET_CONTROL_MODE":
            message.cmd = UAVSetup.SET_CONTROL_MODE
            message.control_state = str(value)
        elif operation == "SET_PX4_MODE":
            message.cmd = UAVSetup.SET_PX4_MODE
            message.px4_mode = str(value)
        else:
            raise TranslationError("unknown Prometheus setup operation")
        self._setup_pub.publish(message)

    def _publish_command(self, operation, value=None):
        message = UAVCommand()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self._odom_frame
        message.Control_Level = UAVCommand.DEFAULT_CONTROL
        if operation == "Init_Pos_Hover":
            message.Agent_CMD = UAVCommand.Init_Pos_Hover
        elif operation == "Current_Pos_Hover":
            message.Agent_CMD = UAVCommand.Current_Pos_Hover
        elif operation == "Land":
            message.Agent_CMD = UAVCommand.Land
        elif operation == "Move_XYZ_POS":
            message.Agent_CMD = UAVCommand.Move
            message.Move_mode = UAVCommand.XYZ_POS
            message.position_ref = list(value[:3])
            message.yaw_ref = value[3]
        else:
            raise TranslationError("unknown Prometheus command operation")
        self._command_id += 1
        message.Command_ID = self._command_id
        self._command_pub.publish(message)

    def _publish_operation(self, operation):
        channel, name, value = operation
        if channel == "setup":
            self._publish_setup(name, value)
        else:
            self._publish_command(name, value)

    def _feedback(self, target=None):
        snapshot = self._snapshot()
        state, _control, odom = snapshot[:3]
        feedback = FlightCommandFeedback()
        if odom is not None:
            feedback.current_pose.header = odom.header
            feedback.current_pose.pose = odom.pose.pose
        if state is not None:
            feedback.native_mode = state.mode
        if target is not None and odom is not None:
            current = odom.pose.pose.position
            feedback.position_error = math.sqrt(
                (current.x - target[0]) ** 2
                + (current.y - target[1]) ** 2
                + (current.z - target[2]) ** 2)
        self._server.publish_feedback(feedback)

    def _abort(self, message):
        self._server.set_aborted(
            FlightCommandResult(success=False, message=message))

    def _succeed(self, message):
        self._server.set_succeeded(
            FlightCommandResult(success=True, message=message))

    def _preempted(self, public_command):
        if public_command == FLY_TO:
            self._publish_command("Current_Pos_Hover")
        self._server.set_preempted(
            FlightCommandResult(success=False, message="preempted"))

    def _wait_native(self, public_command, operation, condition,
                     target=None):
        rate = rospy.Rate(self._publish_rate)
        while not rospy.is_shutdown():
            if self._server.is_preempt_requested():
                self._preempted(public_command)
                return False
            snapshot = self._snapshot()
            if not self._health(snapshot):
                self._abort("Prometheus backend unavailable, stale, or failed")
                return False
            if condition(snapshot):
                return True
            self._publish_operation(operation)
            self._feedback(target)
            rate.sleep()
        self._abort("ROS shutdown")
        return False

    @staticmethod
    def _yaw(pose):
        orientation = pose.orientation
        norm = math.sqrt(
            orientation.x ** 2 + orientation.y ** 2
            + orientation.z ** 2 + orientation.w ** 2)
        if not math.isfinite(norm) or norm <= 0.0:
            raise TranslationError("target orientation is invalid")
        x = orientation.x / norm
        y = orientation.y / norm
        z = orientation.z / norm
        w = orientation.w / norm
        return math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z))

    def _target_in_odom(self, target):
        if not target.header.frame_id:
            raise TranslationError("FLY_TO target frame is empty")
        try:
            transform = self._tf_buffer.lookup_transform(
                self._odom_frame, target.header.frame_id,
                target.header.stamp, rospy.Duration(1.0))
            transformed = do_transform_pose(target, transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as error:
            raise TranslationError("FLY_TO target cannot be transformed") from error
        position = transformed.pose.position
        return (position.x, position.y, position.z,
                self._yaw(transformed.pose))

    def _execute_takeoff(self):
        operations = translation_for(TAKEOFF)
        snapshot = self._snapshot()
        if not self._health(snapshot):
            self._abort("Prometheus backend unavailable, stale, or failed")
            return
        state, _control, odom = snapshot[:3]
        start = odom.pose.pose.position
        takeoff_height = float(rospy.get_param(self._takeoff_height_param))
        if not math.isfinite(takeoff_height) or takeoff_height <= 0.0:
            self._abort("native Prometheus takeoff height is invalid")
            return
        target = (start.x, start.y, start.z + takeoff_height)
        if not self._wait_native(
                TAKEOFF, operations[0], lambda value: value[0].armed):
            return
        if not self._wait_native(
                TAKEOFF, operations[1],
                lambda value: value[1].control_state
                == UAVControlState.COMMAND_CONTROL):
            return

        def airborne(value):
            current_state, _current_control, current_odom = value[:3]
            position = current_odom.pose.pose.position
            return bool(
                current_state.mode == "OFFBOARD"
                and position_complete(
                    (position.x, position.y, position.z), target,
                    current_state.velocity, self._position_tolerance,
                    self._settle_speed))

        rate = rospy.Rate(self._publish_rate)
        while not rospy.is_shutdown():
            if self._server.is_preempt_requested():
                self._preempted(TAKEOFF)
                return
            snapshot = self._snapshot()
            if not self._health(snapshot):
                self._abort("Prometheus backend unavailable, stale, or failed")
                return
            if airborne(snapshot):
                self._succeed("takeoff complete")
                return
            self._publish_operation(operations[2])
            self._publish_operation(operations[3])
            self._feedback(target)
            rate.sleep()
        self._abort("ROS shutdown")

    def _execute_fly_to(self, goal):
        target = self._target_in_odom(goal.target)
        operation = translation_for(FLY_TO, target)[0]

        def arrived(snapshot):
            state, _control, odom = snapshot[:3]
            position = odom.pose.pose.position
            return position_complete(
                (position.x, position.y, position.z), target[:3],
                state.velocity, self._position_tolerance,
                self._settle_speed)

        if self._wait_native(FLY_TO, operation, arrived, target):
            self._succeed("target reached")

    def _execute(self, goal):
        try:
            if goal.command == TAKEOFF:
                self._execute_takeoff()
            elif goal.command == FLY_TO:
                self._execute_fly_to(goal)
            elif goal.command in (HOVER, LAND):
                operation = translation_for(goal.command)[0]
                if goal.command == HOVER:
                    if not self._health():
                        self._abort("Prometheus backend unavailable, stale, or failed")
                        return
                    self._publish_operation(operation)
                    self._feedback()
                    self._succeed("hover commanded")
                elif self._wait_native(
                        LAND, operation,
                        lambda value: not value[0].armed):
                    self._succeed("landing complete")
            else:
                raise TranslationError("unknown public flight command")
        except (TranslationError, KeyError, TypeError, ValueError) as error:
            self._abort(str(error))


def main():
    rospy.init_node("p450_flight_facade")
    PrometheusFlightFacade()
    rospy.spin()


if __name__ == "__main__":
    main()
