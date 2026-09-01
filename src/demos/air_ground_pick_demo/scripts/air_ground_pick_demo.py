#!/usr/bin/env python3
"""Run one sensor-driven P450 handoff and bounded BUNKER approach."""

import json
import math
import os
import sys
import threading
import time

# The ROS executable intentionally shares the package's name.  Put the real
# package directory ahead of the scripts directory so direct execution and
# catkin's generated wrapper cannot turn this file into a shadow module.
_source_root = os.path.realpath(os.path.join(
    os.path.dirname(os.path.realpath(__file__)), "..", "src"))
if os.path.isfile(os.path.join(
        _source_root, "air_ground_pick_demo", "__init__.py")):
    sys.path.insert(0, _source_root)
else:
    for _entry in tuple(sys.path):
        if os.path.isfile(os.path.join(
                _entry, "air_ground_pick_demo", "__init__.py")):
            sys.path.insert(0, _entry)
            break

from geometry_msgs.msg import PoseStamped, Twist
from prometheus_msgs.msg import UAVCommand, UAVControlState, UAVSetup, UAVState
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import tf2_ros

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


class DemoError(RuntimeError):
    """Raised when a bounded runtime phase cannot complete safely."""


class AirGroundPickDemo:
    def __init__(self):
        self._lock = threading.RLock()
        self._uav_state = None
        self._control_state = None
        self._air_pose = None
        self._ground_scan = None
        self._uav_state_received = None
        self._control_state_received = None
        self._air_pose_received = None
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

    def _vector3(self, name):
        raw = self._param(name)
        if not isinstance(raw, (list, tuple)) or len(raw) != 3:
            raise DemoError("~%s must contain three finite values" % name)
        values = []
        for item in raw:
            if type(item) is bool:
                raise DemoError("~%s must contain three finite values" % name)
            try:
                item = float(item)
            except (TypeError, ValueError, OverflowError) as error:
                raise DemoError(
                    "~%s must contain three finite values" % name) from error
            if not math.isfinite(item):
                raise DemoError("~%s must contain three finite values" % name)
            values.append(item)
        return tuple(values)

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

    def _snapshot(self):
        with self._lock:
            return (
                self._uav_state, self._control_state, self._air_pose,
                self._ground_scan, self._uav_state_received,
                self._control_state_received, self._air_pose_received)

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
            return True
        except (DemoError, ApproachError) as error:
            rospy.logerr("air-ground pick demo failed: %s", str(error))
            self._publish_status("FAILED", reason=str(error))
            return False
        finally:
            self._stop_ground()
            self._safe_land()


def main():
    rospy.init_node("air_ground_pick_demo")
    try:
        demo = AirGroundPickDemo()
    except (DemoError, ApproachError) as error:
        rospy.logfatal("invalid air-ground demo configuration: %s", str(error))
        return 2
    return 0 if demo.run() else 1


if __name__ == "__main__":
    raise SystemExit(0 if "--check-imports" in sys.argv else main())
