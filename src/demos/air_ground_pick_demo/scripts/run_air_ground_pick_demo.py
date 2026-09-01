#!/usr/bin/env python3
"""Run one natural Air-Ground observation, grasp, and lift sequence."""

import json
import math
import sys
import threading
import time

import actionlib
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import PoseStamped, Twist
import moveit_commander
from moveit_msgs.msg import RobotState
from prometheus_msgs.msg import UAVCommand, UAVControlState, UAVSetup, UAVState
import rospy
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_pose
import tf2_ros
from trajectory_msgs.msg import JointTrajectoryPoint

from air_ground_pick_demo.approach import (
    ApproachError,
    ApproachLimits,
    clearance_for_status,
    compute_command,
    forward_clearance,
    scan_is_fresh,
    transform_is_fresh,
    travel_distance,
    world_to_base,
)
from air_ground_pick_demo.flight import (
    OneShotFlightSequence,
    message_is_fresh,
    observation_is_fresh,
    position_reached,
    preflight_ready,
    safe_stop_actions,
)
from air_ground_pick_demo.grasp import (
    GraspError,
    check_target_feasibility,
    conservative_jaw_opening,
    contact_sides,
    generate_top_down_grasp,
    zero_terminal_motion,
)


class DemoError(RuntimeError):
    """Raised when a bounded runtime phase cannot complete safely."""


class AirGroundPickDemo:
    def __init__(self):
        self._lock = threading.RLock()
        self._uav_state = None
        self._control_state = None
        self._air_pose = None
        self._ground_scan = None
        self._ground_target_pose = None
        self._joint_state = None
        self._uav_state_received = None
        self._control_state_received = None
        self._air_pose_received = None
        self._ground_pose_received = None
        self._joint_state_received = None
        self._left_contact_received = None
        self._right_contact_received = None
        self._sequence = OneShotFlightSequence()
        self._command_id = 0

        self._status_topic = self._param("status_topic")
        self._uav_state_topic = self._param("uav_state_topic")
        self._uav_control_state_topic = self._param(
            "uav_control_state_topic")
        self._uav_setup_topic = self._param("uav_setup_topic")
        self._uav_command_topic = self._param("uav_command_topic")
        self._air_pose_topic = self._param("air_pose_topic")
        self._ground_scan_topic = self._param("ground_scan_topic")
        self._ground_cmd_topic = self._param("ground_cmd_topic")
        self._ground_pose_topic = self._param("ground_pose_topic")
        self._joint_state_topic = self._param("joint_state_topic")
        self._contact_topic = self._param("contact_topic")
        self._arm_action = self._param("arm_action")
        self._gripper_action = self._param("gripper_action")
        self._world_frame = self._param("world_frame")
        self._ground_base_frame = self._param("ground_base_frame")

        self._preflight_timeout = self._positive("preflight_timeout")
        self._setup_timeout = self._positive("setup_timeout")
        self._takeoff_timeout = self._positive("takeoff_timeout")
        self._view_timeout = self._positive("view_timeout")
        self._observation_timeout = self._positive("observation_timeout")
        self._landing_timeout = self._positive("landing_timeout")
        self._health_max_age = self._positive("flight_health_max_age")
        self._preflight_max_speed = self._nonnegative("preflight_max_speed")
        self._takeoff_height = self._positive("takeoff_height")
        self._view_position = self._vector3("view_position")
        self._view_yaw = self._finite("view_yaw")
        self._flight_position_tolerance = self._positive(
            "flight_position_tolerance")
        self._flight_settle_speed = self._nonnegative(
            "flight_settle_speed")
        self._observation_max_age = self._positive(
            "observation_max_age")

        self._limits = ApproachLimits(
            standoff=self._positive("ground_standoff"),
            distance_tolerance=self._nonnegative(
                "ground_distance_tolerance"),
            heading_tolerance=self._nonnegative(
                "ground_heading_tolerance"),
            turn_in_place_angle=self._positive(
                "ground_turn_in_place_angle"),
            linear_gain=self._positive("ground_linear_gain"),
            angular_gain=self._positive("ground_angular_gain"),
            max_linear=self._positive("max_ground_linear"),
            max_angular=self._positive("max_ground_angular"),
            obstacle_stop_distance=self._positive(
                "obstacle_stop_distance"))
        self._forward_sector_half_angle = self._positive(
            "forward_sector_half_angle")
        self._scan_max_age = self._positive("scan_max_age")
        self._ground_tf_max_age = self._positive("ground_tf_max_age")
        self._ground_timeout = self._positive("ground_timeout")
        self._blocked_timeout = self._positive("blocked_timeout")
        self._max_ground_travel = self._positive("max_ground_travel")
        self._command_period = self._positive("command_period")
        self._stop_publish_count = int(self._param("stop_publish_count"))
        if self._stop_publish_count < 1:
            raise DemoError("stop_publish_count must be positive")

        self._observation_joint_names = self._string_vector(
            "observation_joint_names", 6)
        self._observation_joint_positions = self._finite_vector(
            "observation_joint_positions", 6)
        self._observation_joint_duration = self._positive(
            "observation_joint_duration")
        self._observation_joint_tolerance = self._positive(
            "observation_joint_tolerance")
        self._arm_action_timeout = self._positive("arm_action_timeout")
        self._ground_observation_timeout = self._positive(
            "ground_observation_timeout")
        self._ground_observation_max_age = self._positive(
            "ground_observation_max_age")

        self._move_group_name = self._param("move_group")
        self._end_effector_link = self._param("end_effector_link")
        self._planner_id = self._param("planner_id")
        self._moveit_server_timeout = self._positive(
            "moveit_server_timeout")
        self._planning_time = self._positive("planning_time")
        self._planning_attempts = int(self._param("planning_attempts"))
        if self._planning_attempts < 1:
            raise DemoError("planning_attempts must be positive")
        self._velocity_scaling = self._unit_interval(
            "velocity_scaling", allow_zero=False)
        self._acceleration_scaling = self._unit_interval(
            "acceleration_scaling", allow_zero=False)
        self._planning_position_tolerance = self._positive(
            "planning_position_tolerance")
        self._planning_orientation_tolerance = self._positive(
            "planning_orientation_tolerance")
        self._pose_position_tolerance = self._positive(
            "pose_position_tolerance")
        self._pose_orientation_tolerance = self._positive(
            "pose_orientation_tolerance")
        self._pose_settle_timeout = self._positive("pose_settle_timeout")
        self._cartesian_eef_step = self._positive("cartesian_eef_step")
        self._cartesian_min_fraction = self._unit_interval(
            "cartesian_min_fraction", allow_zero=False)
        self._cartesian_velocity_scaling = self._unit_interval(
            "cartesian_velocity_scaling", allow_zero=False)

        self._target_size = self._finite_vector("target_size", 3)
        self._target_center_height_tolerance = self._positive(
            "target_center_height_tolerance")
        self._maximum_gripper_opening = self._positive(
            "maximum_gripper_opening")
        self._maximum_gripper_joint = self._positive(
            "maximum_gripper_joint")
        self._opening_margin = self._nonnegative("opening_margin")
        self._gripper_open_position = self._nonnegative(
            "gripper_open_position")
        self._gripper_closed_position = self._positive(
            "gripper_closed_position")
        self._minimum_gripper_closed_joint = self._positive(
            "minimum_gripper_closed_joint")
        self._gripper_motion_time = self._positive("gripper_motion_time")
        self._gripper_action_timeout = self._positive(
            "gripper_action_timeout")
        self._gripper_joint_tolerance = self._positive(
            "gripper_joint_tolerance")
        self._pregrasp_height = self._positive("pregrasp_height")
        self._lift_height = self._positive("lift_height")
        self._minimum_lift = self._positive("minimum_lift")
        if self._lift_height < self._minimum_lift:
            raise DemoError("lift_height is below minimum_lift")
        self._finger_pad_lower_edge_offset = self._positive(
            "finger_pad_lower_edge_offset")
        self._contact_overlap = self._positive("contact_overlap")
        self._surface_clearance = self._nonnegative("surface_clearance")
        self._contact_max_age = self._positive("contact_max_age")
        self._contact_timeout = self._positive("contact_timeout")
        self._retention_hold = self._positive("retention_hold")
        self._feasibility = check_target_feasibility(
            self._target_size, self._maximum_gripper_opening,
            self._opening_margin)
        if not self._feasibility.feasible:
            raise DemoError("pick target exceeds the real AG95 opening")

        self._setup_pub = rospy.Publisher(
            self._uav_setup_topic, UAVSetup, queue_size=10)
        self._command_pub = rospy.Publisher(
            self._uav_command_topic, UAVCommand, queue_size=10)
        self._ground_cmd_pub = rospy.Publisher(
            self._ground_cmd_topic, Twist, queue_size=1)
        self._status_pub = rospy.Publisher(
            self._status_topic, String, queue_size=1, latch=True)
        rospy.Subscriber(
            self._uav_state_topic, UAVState, self._uav_state_callback,
            queue_size=10)
        rospy.Subscriber(
            self._uav_control_state_topic, UAVControlState,
            self._control_state_callback, queue_size=10)
        rospy.Subscriber(
            self._air_pose_topic, PoseStamped, self._air_pose_callback,
            queue_size=10)
        rospy.Subscriber(
            self._ground_scan_topic, LaserScan, self._ground_scan_callback,
            queue_size=1)
        rospy.Subscriber(
            self._ground_pose_topic, PoseStamped,
            self._ground_target_pose_callback, queue_size=10)
        rospy.Subscriber(
            self._joint_state_topic, JointState, self._joint_state_callback,
            queue_size=10)
        rospy.Subscriber(
            self._contact_topic, ContactsState, self._contact_callback,
            queue_size=10)
        self._arm_client = actionlib.SimpleActionClient(
            self._arm_action, FollowJointTrajectoryAction)
        self._gripper_client = actionlib.SimpleActionClient(
            self._gripper_action, FollowJointTrajectoryAction)
        self._move_group = None
        self._planning_scene = None
        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

    @staticmethod
    def _param(name):
        if not rospy.has_param("~" + name):
            raise DemoError("missing required parameter ~" + name)
        return rospy.get_param("~" + name)

    def _finite(self, name):
        value = self._param(name)
        if type(value) is bool:
            raise DemoError("~%s must be finite" % name)
        try:
            value = float(value)
        except (TypeError, ValueError, OverflowError) as error:
            raise DemoError("~%s must be finite" % name) from error
        if not math.isfinite(value):
            raise DemoError("~%s must be finite" % name)
        return value

    def _positive(self, name):
        value = self._finite(name)
        if value <= 0.0:
            raise DemoError("~%s must be positive" % name)
        return value

    def _nonnegative(self, name):
        value = self._finite(name)
        if value < 0.0:
            raise DemoError("~%s must be nonnegative" % name)
        return value

    def _unit_interval(self, name, allow_zero=True):
        value = self._finite(name)
        lower_valid = value >= 0.0 if allow_zero else value > 0.0
        if not lower_valid or value > 1.0:
            raise DemoError("~%s must be within the unit interval" % name)
        return value

    def _finite_vector(self, name, length):
        raw = self._param(name)
        if not isinstance(raw, (list, tuple)) or len(raw) != length:
            raise DemoError("~%s must contain %d finite values" %
                            (name, length))
        values = []
        for item in raw:
            if type(item) is bool:
                raise DemoError("~%s must contain finite values" % name)
            try:
                item = float(item)
            except (TypeError, ValueError, OverflowError) as error:
                raise DemoError(
                    "~%s must contain finite values" % name) from error
            if not math.isfinite(item):
                raise DemoError("~%s must contain finite values" % name)
            values.append(item)
        return tuple(values)

    def _string_vector(self, name, length):
        raw = self._param(name)
        if (not isinstance(raw, (list, tuple)) or len(raw) != length or
                any(not isinstance(item, str) or not item for item in raw) or
                len(set(raw)) != length):
            raise DemoError("~%s must contain %d unique names" %
                            (name, length))
        return tuple(raw)

    def _vector3(self, name):
        return self._finite_vector(name, 3)

    def _uav_state_callback(self, message):
        with self._lock:
            self._uav_state = message
            self._uav_state_received = time.monotonic()

    def _control_state_callback(self, message):
        with self._lock:
            self._control_state = message
            self._control_state_received = time.monotonic()

    def _air_pose_callback(self, message):
        with self._lock:
            self._air_pose = message
            self._air_pose_received = time.monotonic()

    def _ground_scan_callback(self, message):
        with self._lock:
            self._ground_scan = message

    def _ground_target_pose_callback(self, message):
        with self._lock:
            self._ground_target_pose = message
            self._ground_pose_received = time.monotonic()

    def _joint_state_callback(self, message):
        with self._lock:
            self._joint_state = message
            self._joint_state_received = time.monotonic()

    def _contact_callback(self, message):
        pairs = tuple(
            (state.collision1_name, state.collision2_name)
            for state in message.states)
        left, right = contact_sides(pairs)
        now = time.monotonic()
        with self._lock:
            if left:
                self._left_contact_received = now
            if right:
                self._right_contact_received = now

    def _snapshot(self):
        with self._lock:
            return (
                self._uav_state, self._control_state, self._air_pose,
                self._ground_scan, self._uav_state_received,
                self._control_state_received, self._air_pose_received)

    def _manipulation_snapshot(self):
        with self._lock:
            return (
                self._ground_target_pose, self._ground_pose_received,
                self._joint_state, self._joint_state_received,
                self._left_contact_received, self._right_contact_received)

    def _publish_status(self, state, **details):
        payload = {
            "state": state,
            "ros_time": rospy.Time.now().to_sec(),
            "wall_time": time.time(),
        }
        payload.update(details)
        self._status_pub.publish(String(data=json.dumps(
            payload, sort_keys=True, allow_nan=False)))
        rospy.loginfo("[air_ground_pick_demo] %s", state)

    def _publish_setup(self, command, arming=False, control_state="",
                       px4_mode=""):
        message = UAVSetup()
        message.header.stamp = rospy.Time.now()
        message.cmd = command
        message.arming = arming
        message.control_state = control_state
        message.px4_mode = px4_mode
        self._setup_pub.publish(message)

    def _publish_command(self, command, position=None, yaw=0.0):
        message = UAVCommand()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "ENU"
        message.Agent_CMD = command
        message.Control_Level = UAVCommand.DEFAULT_CONTROL
        if position is not None:
            message.Move_mode = UAVCommand.XYZ_POS
            message.position_ref = list(position)
            message.yaw_ref = float(yaw)
        self._command_id += 1
        message.Command_ID = self._command_id
        self._command_pub.publish(message)

    @staticmethod
    def _wait_step():
        rospy.rostime.wallsleep(0.05)

    def _repeat_until(self, condition, command, timeout, label,
                      health_check=None):
        deadline = time.monotonic() + timeout
        next_command = 0.0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if condition():
                return
            if health_check is not None and not health_check():
                raise DemoError(label + " flight health lost")
            now = time.monotonic()
            if now >= next_command:
                command()
                next_command = now + self._command_period
            self._wait_step()
        if rospy.is_shutdown():
            raise DemoError("ROS shutdown during " + label)
        raise DemoError(label + " timeout")

    def _preflight_is_ready(self):
        (state, control, _pose, _scan, state_received,
         control_received, _pose_received) = self._snapshot()
        if (state is None or control is None or state_received is None
                or control_received is None):
            return False
        now = time.monotonic()
        return preflight_ready(
            connected=state.connected, odom_valid=state.odom_valid,
            armed=state.armed, failsafe=control.failsafe,
            velocity=state.velocity,
            state_age=now - state_received,
            control_age=now - control_received,
            max_age=self._health_max_age,
            max_speed=self._preflight_max_speed)

    def _wait_preflight(self):
        self._publish_status("PREFLIGHT")
        deadline = time.monotonic() + self._preflight_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self._preflight_is_ready():
                state = self._snapshot()[0]
                return tuple(float(value) for value in state.position)
            self._wait_step()
        if rospy.is_shutdown():
            raise DemoError("ROS shutdown during preflight")
        raise DemoError("preflight timeout")

    def _flight_state_is_fresh(self, snapshot=None, now=None):
        if snapshot is None:
            snapshot = self._snapshot()
        if now is None:
            now = time.monotonic()
        state, control = snapshot[:2]
        if state is None or control is None:
            return False
        return bool(
            self._uav_state_is_current(snapshot, now) and state.odom_valid
            and self._control_state_is_current(snapshot, now)
            and not control.failsafe)

    def _uav_state_is_current(self, snapshot=None, now=None):
        if snapshot is None:
            snapshot = self._snapshot()
        if now is None:
            now = time.monotonic()
        state = snapshot[0]
        state_received = snapshot[4]
        return bool(
            state is not None and state.connected
            and state_received is not None
            and message_is_fresh(
                now - state_received, self._health_max_age))

    def _control_state_is_current(self, snapshot=None, now=None):
        if snapshot is None:
            snapshot = self._snapshot()
        if now is None:
            now = time.monotonic()
        control = snapshot[1]
        control_received = snapshot[5]
        return bool(
            control is not None and control_received is not None
            and message_is_fresh(
                now - control_received, self._health_max_age))

    def _armed_flight_is_healthy(self, snapshot=None, now=None):
        if snapshot is None:
            snapshot = self._snapshot()
        if now is None:
            now = time.monotonic()
        state = snapshot[0]
        return bool(
            state is not None and state.armed
            and self._flight_state_is_fresh(snapshot, now))

    def _command_flight_is_healthy(self, snapshot=None, now=None):
        if snapshot is None:
            snapshot = self._snapshot()
        if now is None:
            now = time.monotonic()
        state, control = snapshot[:2]
        return bool(
            state is not None and state.armed and control is not None
            and control.control_state == UAVControlState.COMMAND_CONTROL
            and self._flight_state_is_fresh(snapshot, now))

    def _arm(self):
        self._sequence.advance("preflight_ready")
        self._publish_status("ARMING")

        def armed():
            snapshot = self._snapshot()
            state = snapshot[0]
            return bool(
                state is not None and state.armed
                and self._flight_state_is_fresh(
                    snapshot, time.monotonic()))

        self._repeat_until(
            armed,
            lambda: self._publish_setup(UAVSetup.ARMING, arming=True),
            self._setup_timeout, "arming",
            health_check=self._flight_state_is_fresh)

    def _enter_command_control(self):
        self._sequence.advance("armed")
        self._publish_status("COMMAND_CONTROL")

        def ready():
            snapshot = self._snapshot()
            state, control = snapshot[:2]
            return bool(
                state is not None and state.armed and control is not None
                and control.control_state == UAVControlState.COMMAND_CONTROL
                and self._flight_state_is_fresh(
                    snapshot, time.monotonic()))

        self._repeat_until(
            ready,
            lambda: self._publish_setup(
                UAVSetup.SET_CONTROL_MODE, control_state="COMMAND_CONTROL"),
            self._setup_timeout, "command control",
            health_check=self._armed_flight_is_healthy)

    def _takeoff(self, home_position):
        self._sequence.advance("command_control_ready")
        self._publish_status("TAKEOFF")
        takeoff_position = (
            home_position[0], home_position[1], self._takeoff_height)

        def airborne():
            snapshot = self._snapshot()
            state = snapshot[0]
            return bool(
                state is not None and state.armed and state.mode == "OFFBOARD"
                and self._command_flight_is_healthy(
                    snapshot, time.monotonic())
                and position_reached(
                    state.position, takeoff_position, state.velocity,
                    self._flight_position_tolerance,
                    self._flight_settle_speed))

        def command_takeoff():
            self._publish_setup(
                UAVSetup.SET_PX4_MODE, px4_mode="OFFBOARD")
            self._publish_command(UAVCommand.Init_Pos_Hover)

        self._repeat_until(
            airborne, command_takeoff, self._takeoff_timeout, "takeoff",
            health_check=self._command_flight_is_healthy)

    def _move_to_view(self):
        self._sequence.advance("airborne")
        self._publish_status("AIR_VIEW")

        def at_view():
            snapshot = self._snapshot()
            state = snapshot[0]
            return bool(
                state is not None and state.armed
                and self._command_flight_is_healthy(
                    snapshot, time.monotonic())
                and position_reached(
                    state.position, self._view_position, state.velocity,
                    self._flight_position_tolerance,
                    self._flight_settle_speed))

        self._repeat_until(
            at_view,
            lambda: self._publish_command(
                UAVCommand.Move, self._view_position, self._view_yaw),
            self._view_timeout, "aerial view",
            health_check=self._command_flight_is_healthy)

    def _observe_and_handoff(self):
        self._sequence.advance("at_view")
        self._publish_status("AIR_OBSERVE")
        observation_started = time.monotonic()
        deadline = observation_started + self._observation_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            snapshot = self._snapshot()
            if not self._command_flight_is_healthy(
                    snapshot, time.monotonic()):
                raise DemoError("aerial observation flight health lost")
            (_state, _control, pose, _scan, _state_received,
             _control_received, pose_received) = snapshot
            if (pose is not None and pose_received is not None
                    and pose_received >= observation_started
                    and observation_is_fresh(
                        pose.header.stamp.to_sec(),
                        rospy.Time.now().to_sec(),
                        self._observation_max_age,
                        pose.header.frame_id, self._world_frame)):
                target = (
                    float(pose.pose.position.x),
                    float(pose.pose.position.y),
                    float(pose.pose.position.z))
                if all(math.isfinite(value) for value in target):
                    self._sequence.advance("fresh_observation")
                    self._publish_status(
                        "AIR_HANDOFF", target_world=list(target),
                        observation_stamp=pose.header.stamp.to_sec())
                    return target
            self._publish_command(UAVCommand.Current_Pos_Hover)
            self._wait_step()
        if rospy.is_shutdown():
            raise DemoError("ROS shutdown during aerial observation")
        raise DemoError("aerial observation timeout")

    def _land(self):
        self._sequence.advance("handoff_published")
        self._publish_status("LANDING")
        self._publish_command(UAVCommand.Current_Pos_Hover)

        def landed():
            snapshot = self._snapshot()
            state = snapshot[0]
            now = time.monotonic()
            return bool(
                state is not None and not state.armed
                and self._uav_state_is_current(snapshot, now))

        self._repeat_until(
            landed, lambda: self._publish_command(UAVCommand.Land),
            self._landing_timeout, "landing")
        self._sequence.advance("landed")

    @staticmethod
    def _yaw_from_quaternion(rotation):
        x = float(rotation.x)
        y = float(rotation.y)
        z = float(rotation.z)
        w = float(rotation.w)
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if not math.isfinite(norm) or norm <= 1e-9:
            raise DemoError("invalid ground base TF quaternion")
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        return math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z))

    def _ground_pose(self):
        transform = self._tf_buffer.lookup_transform(
            self._world_frame, self._ground_base_frame, rospy.Time(0),
            rospy.Duration(0.2))
        if not transform_is_fresh(
                transform.header.stamp.to_sec(),
                rospy.Time.now().to_sec(), self._ground_tf_max_age):
            raise DemoError("ground base TF is stale")
        translation = transform.transform.translation
        yaw = self._yaw_from_quaternion(transform.transform.rotation)
        pose = (float(translation.x), float(translation.y), yaw)
        if not all(math.isfinite(value) for value in pose):
            raise DemoError("ground base TF is nonfinite")
        return pose

    def _publish_ground_command(self, linear_x=0.0, angular_z=0.0):
        message = Twist()
        message.linear.x = float(linear_x)
        message.angular.z = float(angular_z)
        self._ground_cmd_pub.publish(message)

    def _stop_ground(self):
        if not hasattr(self, "_ground_cmd_pub"):
            return
        count = getattr(self, "_stop_publish_count", 1)
        for _index in range(count):
            self._publish_ground_command()
            rospy.rostime.wallsleep(0.02)

    def _approach_ground(self, target_world):
        self._publish_status("GROUND_APPROACH")
        deadline = time.monotonic() + self._ground_timeout
        blocked_since = None
        last_pose = None
        total_travel = 0.0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            try:
                ground_pose = self._ground_pose()
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException, DemoError) as error:
                self._publish_ground_command()
                rospy.logwarn_throttle(
                    1.0, "ground TF unavailable: %s", str(error))
                self._wait_step()
                continue
            if last_pose is not None:
                total_travel += travel_distance(
                    last_pose[:2], ground_pose[:2])
                if total_travel > self._max_ground_travel:
                    raise DemoError("ground travel bound exceeded")
            last_pose = ground_pose

            scan = self._snapshot()[3]
            if scan is None or not scan_is_fresh(
                    scan.header.stamp.to_sec(), rospy.Time.now().to_sec(),
                    self._scan_max_age):
                self._publish_ground_command()
                self._wait_step()
                continue
            try:
                clearance = forward_clearance(
                    scan.ranges, scan.angle_min, scan.angle_increment,
                    self._forward_sector_half_angle,
                    range_min=scan.range_min, range_max=scan.range_max)
                target_base = world_to_base(
                    target_world[:2], ground_pose)
                command = compute_command(
                    target_base[0], target_base[1], clearance, self._limits)
            except ApproachError as error:
                self._publish_ground_command()
                raise DemoError("unsafe ground input: %s" % error) from error

            if command.reached:
                self._stop_ground()
                self._publish_status(
                    "GROUND_STOPPED", ground_travel=total_travel,
                    target_distance=command.distance,
                    forward_clearance=clearance_for_status(
                        command.forward_clearance))
                return
            if command.blocked:
                self._publish_ground_command()
                if blocked_since is None:
                    blocked_since = time.monotonic()
                elif time.monotonic() - blocked_since > self._blocked_timeout:
                    raise DemoError("ground approach blocked by LiDAR")
            else:
                blocked_since = None
                self._publish_ground_command(
                    command.linear_x, command.angular_z)
            self._wait_step()
        if rospy.is_shutdown():
            raise DemoError("ROS shutdown during ground approach")
        raise DemoError("ground approach timeout")

    @staticmethod
    def _joint_position_map(message):
        if (message is None or len(message.name) != len(message.position) or
                not message.name):
            raise DemoError("ground joint state is incomplete")
        positions = {
            name: float(position)
            for name, position in zip(message.name, message.position)}
        if not all(math.isfinite(value) for value in positions.values()):
            raise DemoError("ground joint state is nonfinite")
        return positions

    def _current_joint_positions(self, maximum_age=1.0):
        snapshot = self._manipulation_snapshot()
        message = snapshot[2]
        received = snapshot[3]
        if (message is None or received is None or
                time.monotonic() - received > maximum_age):
            raise DemoError("ground joint state is stale")
        return self._joint_position_map(message)

    def _robot_state_from_joint_feedback(self, maximum_age=1.0):
        snapshot = self._manipulation_snapshot()
        message = snapshot[2]
        received = snapshot[3]
        if (message is None or received is None or
                time.monotonic() - received > maximum_age):
            raise DemoError("ground joint state is stale")
        positions = self._joint_position_map(message)
        if not set(self._observation_joint_names).issubset(positions):
            raise DemoError("ground arm joint state is incomplete")
        state = RobotState()
        state.joint_state.header.stamp = rospy.Time.now()
        state.joint_state.name = list(message.name)
        state.joint_state.position = [
            positions[name] for name in message.name]
        return state

    def _max_arm_joint_speed(self):
        message = self._manipulation_snapshot()[2]
        if (message is None or len(message.name) != len(message.velocity)):
            return math.nan, "unavailable"
        velocities = dict(zip(message.name, message.velocity))
        measured = [
            (abs(float(velocities[name])), name)
            for name in self._observation_joint_names
            if name in velocities and math.isfinite(float(velocities[name]))]
        return max(measured) if measured else (math.nan, "unavailable")

    def _wait_joint_target(self, names, targets, tolerance, timeout, label):
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            try:
                positions = self._current_joint_positions()
                if all(name in positions and
                       abs(positions[name] - target) <= tolerance
                       for name, target in zip(names, targets)):
                    return positions
            except DemoError:
                pass
            self._wait_step()
        raise DemoError(label + " measured joint target timeout")

    @staticmethod
    def _trajectory_goal(names, positions, duration):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = list(names)
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.velocities = [0.0] * len(positions)
        point.time_from_start = rospy.Duration(duration)
        goal.trajectory.points = [point]
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
        return goal

    def _execute_trajectory(
            self, client, names, positions, duration, timeout, label,
            accept_contact_stall=False):
        if not client.wait_for_server(rospy.Duration(timeout)):
            raise DemoError(label + " controller action is unavailable")
        client.send_goal(self._trajectory_goal(names, positions, duration))
        if not client.wait_for_result(rospy.Duration(timeout)):
            client.cancel_goal()
            raise DemoError(label + " controller action timed out")
        state = client.get_state()
        if state == GoalStatus.SUCCEEDED:
            return
        if accept_contact_stall:
            try:
                joint = self._current_joint_positions()[names[0]]
            except (DemoError, KeyError):
                joint = -math.inf
            if (joint >= self._minimum_gripper_closed_joint and
                    self._bilateral_contact_current()):
                rospy.loginfo(
                    "AG95 contact stall accepted at %.4f rad", joint)
                return
        raise DemoError(
            "%s controller action failed: state=%s status=%s" %
            (label, state, client.get_goal_status_text()))

    def _open_gripper(self):
        self._execute_trajectory(
            self._gripper_client, ("left_outer_knuckle_joint",),
            (self._gripper_open_position,), self._gripper_motion_time,
            self._gripper_action_timeout, "AG95 open")
        positions = self._wait_joint_target(
            ("left_outer_knuckle_joint",),
            (self._gripper_open_position,), self._gripper_joint_tolerance,
            self._gripper_action_timeout, "AG95 open")
        opening = conservative_jaw_opening(
            positions["left_outer_knuckle_joint"],
            self._maximum_gripper_opening, self._maximum_gripper_joint)
        if opening <= self._feasibility.required_opening:
            raise DemoError("measured AG95 opening is too small for target")
        return opening

    def _move_to_ground_observation(self):
        self._execute_trajectory(
            self._arm_client, self._observation_joint_names,
            self._observation_joint_positions,
            self._observation_joint_duration, self._arm_action_timeout,
            "AUBO observation")
        self._wait_joint_target(
            self._observation_joint_names,
            self._observation_joint_positions,
            self._observation_joint_tolerance, self._arm_action_timeout,
            "AUBO observation")

    def _target_from_pose(self, pose):
        target = (
            float(pose.pose.position.x), float(pose.pose.position.y),
            float(pose.pose.position.z),
            self._yaw_from_quaternion(pose.pose.orientation))
        if not all(math.isfinite(value) for value in target):
            raise DemoError("near-field target pose is nonfinite")
        expected_center_z = 0.5 * self._target_size[2]
        if abs(target[2] - expected_center_z) > \
                self._target_center_height_tolerance:
            raise DemoError(
                "near-field target height is inconsistent: "
                "measured=%.4f expected=%.4f tolerance=%.4f" %
                (target[2], expected_center_z,
                 self._target_center_height_tolerance))
        return target

    def _observe_ground_target(self):
        self._publish_status("GROUND_OBSERVE")
        opening = self._open_gripper()
        self._move_to_ground_observation()
        started = time.monotonic()
        deadline = started + self._ground_observation_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            pose, received = self._manipulation_snapshot()[:2]
            if (pose is not None and received is not None and
                    received >= started and observation_is_fresh(
                        pose.header.stamp.to_sec(),
                        rospy.Time.now().to_sec(),
                        self._ground_observation_max_age,
                        pose.header.frame_id, self._world_frame)):
                target = self._target_from_pose(pose)
                self._publish_status(
                    "GROUND_REFINED", target_world=list(target),
                    observation_stamp=pose.header.stamp.to_sec(),
                    measured_gripper_opening=opening)
                return pose, target
            self._wait_step()
        raise DemoError("near-field D435 observation timeout")

    @staticmethod
    def _pose_message(cartesian_pose, frame_id, stamp):
        message = PoseStamped()
        message.header.frame_id = frame_id
        message.header.stamp = stamp
        message.pose.position.x = cartesian_pose.position[0]
        message.pose.position.y = cartesian_pose.position[1]
        message.pose.position.z = cartesian_pose.position[2]
        message.pose.orientation.x = cartesian_pose.orientation[0]
        message.pose.orientation.y = cartesian_pose.orientation[1]
        message.pose.orientation.z = cartesian_pose.orientation[2]
        message.pose.orientation.w = cartesian_pose.orientation[3]
        return message

    def _transform_pose(self, pose, target_frame, use_latest=False):
        if pose.header.frame_id == target_frame:
            result = PoseStamped()
            result.header = pose.header
            result.pose = pose.pose
            return result
        transform = self._tf_buffer.lookup_transform(
            target_frame, pose.header.frame_id,
            rospy.Time(0) if use_latest else pose.header.stamp,
            rospy.Duration(0.5))
        result = do_transform_pose(pose, transform)
        result.header.frame_id = target_frame
        return result

    def _initialize_moveit(self):
        if self._move_group is not None:
            return self._move_group
        group = moveit_commander.MoveGroupCommander(
            self._move_group_name, wait_for_servers=self._moveit_server_timeout)
        group.set_end_effector_link(self._end_effector_link)
        group.set_planner_id(self._planner_id)
        group.set_planning_time(self._planning_time)
        group.set_num_planning_attempts(self._planning_attempts)
        group.set_max_velocity_scaling_factor(self._velocity_scaling)
        group.set_max_acceleration_scaling_factor(self._acceleration_scaling)
        group.set_goal_position_tolerance(
            self._planning_position_tolerance)
        group.set_goal_orientation_tolerance(
            self._planning_orientation_tolerance)
        if set(group.get_active_joints()) != set(
                self._observation_joint_names):
            raise DemoError("MoveIt manipulator joint set is inconsistent")
        if group.get_end_effector_link() != self._end_effector_link:
            raise DemoError("MoveIt end-effector link is inconsistent")

        self._planning_scene = moveit_commander.PlanningSceneInterface(
            synchronous=True)
        floor = PoseStamped()
        floor.header.frame_id = self._world_frame
        floor.header.stamp = rospy.Time(0)
        floor.pose.orientation.w = 1.0
        floor = self._transform_pose(
            floor, group.get_planning_frame(), use_latest=True)
        self._planning_scene.add_plane("air_ground_floor", floor)
        self._move_group = group
        return group

    @staticmethod
    def _quaternion_error(first, second):
        a = (
            first.x, first.y, first.z, first.w)
        b = (
            second.x, second.y, second.z, second.w)
        a_norm = math.sqrt(sum(value * value for value in a))
        b_norm = math.sqrt(sum(value * value for value in b))
        if a_norm <= 1e-12 or b_norm <= 1e-12:
            return math.inf
        dot = abs(sum(x * y for x, y in zip(a, b)) / (a_norm * b_norm))
        return 2.0 * math.acos(min(1.0, max(0.0, dot)))

    def _verify_tcp_pose(self, target, label):
        deadline = time.monotonic() + self._pose_settle_timeout
        position_error = math.inf
        orientation_error = math.inf
        fresh = False
        while not rospy.is_shutdown():
            transform = self._tf_buffer.lookup_transform(
                target.header.frame_id, self._end_effector_link,
                rospy.Time(0), rospy.Duration(0.5))
            actual = PoseStamped()
            actual.header.frame_id = target.header.frame_id
            actual.header.stamp = transform.header.stamp
            actual.pose.position.x = transform.transform.translation.x
            actual.pose.position.y = transform.transform.translation.y
            actual.pose.position.z = transform.transform.translation.z
            actual.pose.orientation = transform.transform.rotation
            stamp = actual.header.stamp.to_sec()
            fresh = bool(
                stamp <= 0.0 or transform_is_fresh(
                    stamp, rospy.Time.now().to_sec(),
                    self._ground_tf_max_age))
            if fresh:
                dx = actual.pose.position.x - target.pose.position.x
                dy = actual.pose.position.y - target.pose.position.y
                dz = actual.pose.position.z - target.pose.position.z
                position_error = math.sqrt(dx * dx + dy * dy + dz * dz)
                orientation_error = self._quaternion_error(
                    actual.pose.orientation, target.pose.orientation)
                if (position_error <= self._pose_position_tolerance and
                        orientation_error <=
                        self._pose_orientation_tolerance):
                    return actual
            if time.monotonic() >= deadline:
                break
            self._wait_step()
        if not fresh:
            raise DemoError(label + " TCP transform did not become fresh")
        raise DemoError(
            "%s TCP pose error position=%.4f orientation=%.4f" %
            (label, position_error, orientation_error))

    def _execute_pregrasp(self, target):
        group = self._move_group
        group.set_start_state_to_current_state()
        group.set_pose_target(target, self._end_effector_link)
        planned = group.plan()
        success = bool(planned[0]) if isinstance(planned, tuple) else True
        trajectory = planned[1] if isinstance(planned, tuple) else planned
        if (not success or
                not trajectory.joint_trajectory.points):
            group.clear_pose_targets()
            raise DemoError("MoveIt pregrasp planning failed")
        if not group.execute(trajectory, wait=True):
            group.clear_pose_targets()
            raise DemoError("MoveIt pregrasp execution failed")
        group.stop()
        group.clear_pose_targets()
        return self._verify_tcp_pose(target, "pregrasp")

    def _execute_cartesian(self, target, label):
        group = self._move_group
        trajectory, fraction = group.compute_cartesian_path(
            [target.pose], self._cartesian_eef_step, True)
        if (not math.isfinite(fraction) or
                fraction < self._cartesian_min_fraction or
                not trajectory.joint_trajectory.points):
            raise DemoError(
                "%s Cartesian path fraction %.3f is insufficient" %
                (label, fraction))
        trajectory = group.retime_trajectory(
            self._robot_state_from_joint_feedback(), trajectory,
            self._cartesian_velocity_scaling,
            self._cartesian_velocity_scaling)
        if not trajectory.joint_trajectory.points:
            raise DemoError(label + " Cartesian retiming failed")
        trajectory = zero_terminal_motion(trajectory)
        if not group.execute(trajectory, wait=True):
            max_joint_speed, speed_joint = self._max_arm_joint_speed()
            raise DemoError(
                "%s Cartesian execution failed: max_joint_speed=%.4f "
                "joint=%s" % (label, max_joint_speed, speed_joint))
        group.stop()
        return self._verify_tcp_pose(target, label)

    def _bilateral_contact_current(self, now=None):
        snapshot = self._manipulation_snapshot()
        left = snapshot[4]
        right = snapshot[5]
        current = time.monotonic() if now is None else now
        return bool(
            left is not None and right is not None and
            current - left <= self._contact_max_age and
            current - right <= self._contact_max_age)

    def _wait_bilateral_contact(self):
        deadline = time.monotonic() + self._contact_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self._bilateral_contact_current():
                return
            self._wait_step()
        raise DemoError("AG95 did not establish bilateral target contact")

    def _hold_bilateral_contact(self):
        deadline = time.monotonic() + self._retention_hold
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if not self._bilateral_contact_current():
                raise DemoError("AG95 lost bilateral contact during lift hold")
            self._wait_step()

    def _close_gripper(self):
        self._execute_trajectory(
            self._gripper_client, ("left_outer_knuckle_joint",),
            (self._gripper_closed_position,), self._gripper_motion_time,
            self._gripper_action_timeout, "AG95 close",
            accept_contact_stall=True)
        self._wait_bilateral_contact()
        positions = self._current_joint_positions()
        if positions.get("left_outer_knuckle_joint", -math.inf) < \
                self._minimum_gripper_closed_joint:
            raise DemoError("AG95 did not close around the target")

    def _pick_and_lift(self, sensor_pose, target):
        generated = generate_top_down_grasp(
            target, self._target_size, self._pregrasp_height,
            self._lift_height, self._finger_pad_lower_edge_offset,
            self._contact_overlap, self._surface_clearance)
        group = self._initialize_moveit()
        planning_frame = group.get_planning_frame()
        messages = tuple(
            self._transform_pose(
                self._pose_message(pose, self._world_frame,
                                   sensor_pose.header.stamp),
                planning_frame)
            for pose in (generated.pregrasp, generated.grasp, generated.lift))
        pregrasp, grasp, lift = messages

        self._publish_status("PREGRASP")
        self._execute_pregrasp(pregrasp)
        self._publish_status("GRASP")
        grasp_actual = self._execute_cartesian(grasp, "grasp approach")
        self._close_gripper()
        self._publish_status("LIFTING")
        lift_actual = self._execute_cartesian(lift, "lift")
        measured_lift = (
            lift_actual.pose.position.z - grasp_actual.pose.position.z)
        if measured_lift < self._minimum_lift:
            raise DemoError("AUBO TCP did not complete the minimum lift")
        self._hold_bilateral_contact()
        self._publish_status(
            "LIFT", tcp_lift=measured_lift, bilateral_contact=True,
            target_world=list(target))

    def _safe_land(self):
        if not hasattr(self, "_command_pub"):
            return
        deadline = time.monotonic() + self._landing_timeout
        next_command = 0.0
        hold_sent = False
        snapshot = None
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            snapshot = self._snapshot()
            state, control = snapshot[:2]
            if state is None:
                return
            now = time.monotonic()
            if (self._uav_state_is_current(snapshot, now)
                    and not state.armed):
                return
            if now < next_command:
                self._wait_step()
                continue
            actions = safe_stop_actions(
                armed=True,
                command_control=bool(
                    self._control_state_is_current(snapshot, now)
                    and control is not None and
                    control.control_state ==
                    UAVControlState.COMMAND_CONTROL))
            for action in actions:
                if action == "HOLD" and not hold_sent:
                    self._publish_command(UAVCommand.Current_Pos_Hover)
                    hold_sent = True
                elif action == "LAND":
                    self._publish_command(UAVCommand.Land)
                elif action == "AUTO_LAND":
                    self._publish_setup(
                        UAVSetup.SET_PX4_MODE, px4_mode="AUTO.LAND")
            next_command = now + self._command_period
            self._wait_step()
        state = None if snapshot is None else snapshot[0]
        now = time.monotonic()
        if (not rospy.is_shutdown()
                and (state is None
                     or not self._uav_state_is_current(snapshot, now)
                     or state.armed)):
            rospy.logerr("emergency landing did not disarm before timeout")

    def run(self):
        try:
            home_position = self._wait_preflight()
            self._arm()
            self._enter_command_control()
            self._takeoff(home_position)
            self._move_to_view()
            target_world = self._observe_and_handoff()
            self._land()
            self._approach_ground(target_world)
            sensor_pose, refined_target = self._observe_ground_target()
            self._pick_and_lift(sensor_pose, refined_target)
            return True
        except (DemoError, ApproachError, GraspError,
                moveit_commander.MoveItCommanderException,
                tf2_ros.TransformException) as error:
            rospy.logerr("air-ground pick demo failed: %s", str(error))
            self._publish_status("FAILED", reason=str(error))
            return False
        finally:
            self._stop_ground()
            self._safe_land()


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("air_ground_pick_demo")
    try:
        demo = AirGroundPickDemo()
    except (DemoError, ApproachError, GraspError,
            moveit_commander.MoveItCommanderException) as error:
        rospy.logfatal("invalid air-ground demo configuration: %s", str(error))
        moveit_commander.roscpp_shutdown()
        return 2
    try:
        return 0 if demo.run() else 1
    finally:
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    raise SystemExit(0 if "--check-imports" in sys.argv else main())
