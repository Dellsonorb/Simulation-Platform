#!/usr/bin/env python3
"""Fly the P450 through the fixed M1 viewpoints using Prometheus control."""

import json
import math
import threading
import time

import numpy as np
import rospy
from mavros_msgs.msg import State as MavrosState
from prometheus_msgs.msg import UAVCommand, UAVControlState, UAVSetup, UAVState
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

from brick_aerial_perception.viewpoint_mission import MissionState, ViewpointMission


class AerialViewpointMissionNode:
    def __init__(self):
        self.uav_id = int(rospy.get_param("~uav_id", 1))
        viewpoints = rospy.get_param("~viewpoints")
        self.mission = ViewpointMission(
            viewpoints,
            rospy.get_param("~arrival_tolerance", 0.20),
            rospy.get_param("~yaw_tolerance", 0.14),
            rospy.get_param("~settle_time", 1.5),
            rospy.get_param("~waypoint_timeout", 35.0),
            rospy.get_param("~max_settle_linear_speed", 0.10),
            rospy.get_param("~max_settle_angular_speed", 0.10),
        )
        self.sample_duration = float(rospy.get_param("~sample_duration", 3.0))
        self.hold_after_last_sample = bool(
            rospy.get_param("~hold_after_last_sample", False)
        )
        self.takeoff_height = float(rospy.get_param("~takeoff_height", 1.5))
        self.takeoff_tolerance = float(rospy.get_param("~takeoff_tolerance", 0.25))
        self.setup_timeout = float(rospy.get_param("~setup_timeout", 30.0))
        self.takeoff_timeout = float(rospy.get_param("~takeoff_timeout", 30.0))
        self.health_timeout = float(rospy.get_param("~health_timeout", 1.5))
        self.clock_start_timeout = float(
            rospy.get_param("~clock_start_timeout", 5.0))
        self.autostart = bool(rospy.get_param("~autostart", True))
        self.flight_state = "PREFLIGHT" if self.autostart else "IDLE"
        self.sequence_generation = 1 if self.autostart else 0
        self.sequence_ready = False
        self.viewpoint_ids = [item["id"] for item in viewpoints]
        self.phase_started = rospy.Time.now().to_sec()
        self.ready_started = None
        self.takeoff_stable_started = None
        self.sample_started = None
        self.last_command = 0.0
        self.command_id = 0
        self.uav_state = None
        self.control_state = None
        self.mavros_state = None
        self.last_uav_state = 0.0
        self.last_control_state = 0.0
        self.last_mavros_state = 0.0
        self.lock = threading.RLock()

        prefix = "/uav{}/prometheus/".format(self.uav_id)
        self.command_pub = rospy.Publisher(prefix + "command", UAVCommand, queue_size=10)
        self.setup_pub = rospy.Publisher(prefix + "setup", UAVSetup, queue_size=10)
        self.state_pub = rospy.Publisher("/m1/viewpoint_state", String, queue_size=1, latch=True)
        self.sequence_pub = rospy.Publisher(
            "/m1/viewpoint_sequence_status", String, queue_size=1, latch=True
        )
        rospy.Subscriber(prefix + "state", UAVState, self._uav_state_cb, queue_size=10)
        rospy.Subscriber(prefix + "control_state", UAVControlState,
                         self._control_state_cb, queue_size=10)
        rospy.Subscriber("/uav{}/mavros/state".format(self.uav_id), MavrosState,
                         self._mavros_state_cb, queue_size=10)
        rospy.Service("~start", Trigger, self._start_cb)
        rospy.Service("~abort", Trigger, self._abort_cb)
        self._publish_state(self.flight_state)
        self.timer = rospy.Timer(rospy.Duration(0.1), self._tick)

    def _uav_state_cb(self, message):
        self.uav_state = message
        self.last_uav_state = rospy.Time.now().to_sec()

    def _control_state_cb(self, message):
        self.control_state = message
        self.last_control_state = rospy.Time.now().to_sec()

    def _mavros_state_cb(self, message):
        self.mavros_state = message
        self.last_mavros_state = rospy.Time.now().to_sec()

    def _publish_state(self, state):
        self.state_pub.publish(String(data=state))
        self._publish_sequence_status()
        rospy.loginfo("[m1_viewpoints] %s", state)

    def _publish_sequence_status(self):
        active_viewpoint = None
        if ":" in self.flight_state:
            active_viewpoint = self.flight_state.split(":", 1)[1] or None
        payload = {
            "generation": int(self.sequence_generation),
            "state": self.flight_state,
            "viewpoint_ids": list(self.viewpoint_ids),
            "completed_viewpoints": list(self.mission.completed_ids),
            "active_viewpoint": active_viewpoint,
            "ready": bool(self.sequence_ready),
            "ros_time": rospy.Time.now().to_sec(),
            "wall_time": time.time(),
        }
        self.sequence_pub.publish(String(data=json.dumps(
            payload, sort_keys=True, allow_nan=False
        )))

    def _transition(self, state, now):
        self.flight_state = state
        self.phase_started = now
        self._publish_state(state)

    def _wait_for_positive_ros_time(self):
        deadline = time.monotonic() + self.clock_start_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if rospy.Time.now().to_nsec() > 0:
                return True
            rospy.rostime.wallsleep(0.01)
        return False

    def _start_cb(self, _request):
        # A child roslaunch can advertise this service before its first
        # /clock callback.  Capturing phase_started=0 in an already-running
        # Gazebo world makes the next timer tick consume most or all of the
        # preflight timeout.  Keep the service fail-closed until this process
        # has observed the simulation epoch; wall time is used only to bound
        # that wait because ROS time is deliberately unavailable here.
        if not self._wait_for_positive_ros_time():
            return TriggerResponse(False, "ROS_CLOCK_NOT_READY")
        with self.lock:
            if self.flight_state not in ("IDLE", "COMPLETE", "FAILED"):
                return TriggerResponse(False, "mission already active")
            self.ready_started = None
            self.takeoff_stable_started = None
            self.sample_started = None
            self.sequence_generation += 1
            self.sequence_ready = False
            self._transition("PREFLIGHT", rospy.Time.now().to_sec())
            return TriggerResponse(True, "M1 viewpoint mission started")

    def _abort_cb(self, _request):
        with self.lock:
            self._fail("operator_abort", rospy.Time.now().to_sec())
            return TriggerResponse(True, "safe landing requested")

    def _publish_setup(self, command, arming=False, control_state="", px4_mode=""):
        message = UAVSetup()
        message.header.stamp = rospy.Time.now()
        message.cmd = command
        message.arming = arming
        message.control_state = control_state
        message.px4_mode = px4_mode
        self.setup_pub.publish(message)

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
        self.command_id += 1
        message.Command_ID = self.command_id
        self.command_pub.publish(message)

    def _fresh(self, stamp, now):
        return stamp > 0.0 and now - stamp <= self.health_timeout

    def _preflight_ready(self, now):
        return bool(
            self.uav_state is not None and self.control_state is not None
            and self.mavros_state is not None and self.mavros_state.connected
            and not self.uav_state.armed and self.uav_state.odom_valid
            and self.uav_state.gps_status >= UAVState.GPS_FIX_TYPE_3D_FIX
            and not self.control_state.failsafe
            and self._fresh(self.last_uav_state, now)
            and self._fresh(self.last_control_state, now)
            and self._fresh(self.last_mavros_state, now)
        )

    def _health_ok(self, now):
        return bool(
            self.uav_state is not None and self.control_state is not None
            and self.mavros_state is not None and self.mavros_state.connected
            and self.uav_state.odom_valid and not self.control_state.failsafe
            and self._fresh(self.last_uav_state, now)
            and self._fresh(self.last_control_state, now)
            and self._fresh(self.last_mavros_state, now)
        )

    def _fail(self, reason, now):
        if self.flight_state in ("COMPLETE", "FAILED"):
            return
        if self.control_state is not None and (
                self.control_state.control_state == UAVControlState.COMMAND_CONTROL):
            self._publish_command(UAVCommand.Current_Pos_Hover)
            self._publish_command(UAVCommand.Land)
        else:
            self._publish_setup(UAVSetup.SET_PX4_MODE, px4_mode="AUTO.LAND")
        self.flight_state = "FAILED"
        self.sequence_ready = False
        self.phase_started = now
        self._publish_state("FAILED:" + reason)

    def _tick(self, _event):
        with self.lock:
            now = rospy.Time.now().to_sec()
            if self.flight_state in ("IDLE", "COMPLETE", "FAILED"):
                return
            if self.flight_state not in ("PREFLIGHT", "ARMING") and not self._health_ok(now):
                self._fail("flight_health_lost", now)
                return
            if self.flight_state == "PREFLIGHT":
                if now - self.phase_started > self.setup_timeout:
                    self._fail("preflight_timeout", now)
                elif self._preflight_ready(now):
                    if self.ready_started is None:
                        self.ready_started = now
                    elif now - self.ready_started >= 1.0:
                        self._transition("ARMING", now)
                else:
                    self.ready_started = None
                return
            if self.flight_state == "ARMING":
                if self.uav_state is not None and self.uav_state.armed:
                    self._transition("ENTER_COMMAND_CONTROL", now)
                elif now - self.phase_started > self.setup_timeout:
                    self._fail("arming_timeout", now)
                elif now - self.last_command >= 1.0:
                    self._publish_setup(UAVSetup.ARMING, arming=True)
                    self.last_command = now
                return
            if self.flight_state == "ENTER_COMMAND_CONTROL":
                if self.control_state.control_state == UAVControlState.COMMAND_CONTROL:
                    self._transition("TAKEOFF", now)
                    self._publish_command(UAVCommand.Init_Pos_Hover)
                    self.last_command = now
                elif now - self.phase_started > self.setup_timeout:
                    self._fail("control_mode_timeout", now)
                elif now - self.last_command >= 1.0:
                    self._publish_setup(UAVSetup.SET_CONTROL_MODE,
                                        control_state="COMMAND_CONTROL")
                    self.last_command = now
                return
            if self.flight_state == "TAKEOFF":
                if now - self.phase_started > self.takeoff_timeout:
                    self._fail("takeoff_timeout", now)
                    return
                if now - self.last_command >= 0.5:
                    self._publish_command(UAVCommand.Init_Pos_Hover)
                    self.last_command = now
                height_error = abs(float(self.uav_state.position[2]) - self.takeoff_height)
                if height_error <= self.takeoff_tolerance:
                    if self.takeoff_stable_started is None:
                        self.takeoff_stable_started = now
                    elif now - self.takeoff_stable_started >= 1.0:
                        self.mission.start(now)
                        self._transition("MOVING:" + self.mission.current["id"], now)
                else:
                    self.takeoff_stable_started = None
                return
            if self.flight_state.startswith("MOVING:"):
                target = self.mission.current
                if now - self.last_command >= 0.5:
                    self._publish_command(UAVCommand.Move, target["position"], target["yaw"])
                    self.last_command = now
                linear_speed = float(np.linalg.norm(self.uav_state.velocity))
                angular_speed = float(np.linalg.norm(self.uav_state.attitude_rate))
                event = self.mission.observe(
                    self.uav_state.position, self.uav_state.attitude[2],
                    linear_speed, angular_speed, now,
                )
                if event == "waypoint_timeout":
                    self._fail("waypoint_timeout:" + target["id"], now)
                elif event == "sampling_started":
                    self.sample_started = now
                    self._transition("SAMPLING:" + target["id"], now)
                return
            if self.flight_state.startswith("SAMPLING:"):
                self._publish_command(UAVCommand.Current_Pos_Hover)
                if self.sequence_ready:
                    return
                if now - self.sample_started < self.sample_duration:
                    return
                event = self.mission.finish_sample(
                    now, hold_on_final=self.hold_after_last_sample
                )
                if event == "next_viewpoint":
                    self._transition("MOVING:" + self.mission.current["id"], now)
                elif event == "sequence_ready":
                    self.sequence_ready = True
                    self._publish_sequence_status()
                    return
                elif event == "mission_complete":
                    self._publish_command(UAVCommand.Land)
                    self._transition("LANDING", now)
                else:
                    self._fail("invalid_sampling_transition", now)
                return
            if self.flight_state == "LANDING":
                if now - self.last_command >= 1.0:
                    self._publish_command(UAVCommand.Land)
                    self.last_command = now
                if self.uav_state is not None and not self.uav_state.armed:
                    self._transition("COMPLETE", now)


if __name__ == "__main__":
    rospy.init_node("m1_aerial_viewpoint_mission")
    AerialViewpointMissionNode()
    rospy.spin()
