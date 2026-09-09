#!/usr/bin/env python3
"""Run one natural air observation, ground approach, grasp, and lift."""

import copy
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
from geometry_msgs.msg import PointStamped, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import moveit_commander
from moveit_msgs.msg import PlanningSceneComponents, RobotState
from moveit_msgs.srv import (
    ApplyPlanningScene, ApplyPlanningSceneRequest,
    GetCartesianPath, GetCartesianPathRequest,
    GetPlanningScene, GetPlanningSceneRequest,
    GetStateValidity, GetStateValidityRequest,
)
from prometheus_msgs.msg import UAVState
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_geometry_msgs import do_transform_pose
import tf2_ros
from trajectory_msgs.msg import JointTrajectoryPoint

from air_ground_pick_demo.approach import (
    ApproachError,
    arrival_within_tolerance,
    compute_heading_goal,
    compute_staged_candidate_goals,
    compute_staged_standoff_goals,
    motion_required,
    transform_is_fresh,
    travel_distance,
)
from air_ground_pick_demo.flight import (
    native_state_ready,
    observation_is_fresh,
)
from air_ground_pick_demo.grasp import (
    GraspError,
    check_target_feasibility,
    conservative_jaw_opening,
    generate_top_down_grasp,
    zero_terminal_motion,
)
from air_ground_pick_demo import manipulation_scene


class DemoError(RuntimeError):
    pass


class AirGroundPickDemo:
    def __init__(self):
        # Generated action imports are intentionally local so source-only
        # import checks do not require a previously installed build tree.
        from robot_runtime_interfaces.msg import (
            FlightCommandAction,
            FlightCommandGoal,
        )

        self._lock = threading.RLock()
        self._uav_state = None
        self._air_pose = None
        self._ground_target_pose = None
        self._ground_surface_cue = None
        self._ground_observation_mode = rospy.get_param('~ground_observation_mode', 'legacy_pregrasp')
        self._full_robot_manipulation = rospy.get_param("~full_robot_manipulation", False)
        if type(self._full_robot_manipulation) is not bool:
            raise DemoError("~full_robot_manipulation must be boolean")
        self._joint_state = None
        self._grasp_confirmed = False
        self._uav_state_received = None
        self._air_pose_received = None
        self._ground_pose_received = None
        self._joint_state_received = None
        self._grasp_confirmation_received = None
        self._flight_started = False
        self._landed = False

        self._status_topic = self._param("status_topic")
        self._uav_state_topic = self._param("uav_state_topic")
        self._flight_action = self._param("flight_action")
        self._air_pose_topic = self._param("air_pose_topic")
        self._ground_navigation_action = self._param(
            "ground_navigation_action")
        self._ground_stop_service = self._param("ground_stop_service")
        self._ground_pose_topic = self._param("ground_pose_topic")
        self._joint_state_topic = self._param("joint_state_topic")
        self._grasp_confirmed_topic = self._param("grasp_confirmed_topic")
        self._arm_action = self._param("arm_action")
        self._gripper_action = self._param("gripper_action")
        self._map_frame = self._param("map_frame")
        self._ground_base_frame = self._param("ground_base_frame")
        self._placement_mode = self._param("placement_mode")
        if self._placement_mode not in ("standoff", "rm4d"):
            raise DemoError("~placement_mode must be standoff or rm4d")
        self._rm4d_service = self._param("rm4d_service")
        self._rm4d_grasp_id = self._param("rm4d_grasp_id")
        if (not isinstance(self._rm4d_grasp_id, str) or
                not self._rm4d_grasp_id):
            raise DemoError("~rm4d_grasp_id must be a non-empty string")
        self._rm4d_top_k = self._param("rm4d_top_k")
        if type(self._rm4d_top_k) is not int or self._rm4d_top_k <= 0:
            raise DemoError("~rm4d_top_k must be a positive integer")
        self._rm4d_service_timeout = self._positive(
            "rm4d_service_timeout")
        self._rm4d_pregrasp_plan_attempts = self._param(
            "rm4d_pregrasp_plan_attempts")
        if (type(self._rm4d_pregrasp_plan_attempts) is not int or
                self._rm4d_pregrasp_plan_attempts <= 0):
            raise DemoError(
                "~rm4d_pregrasp_plan_attempts must be a positive integer")

        self._preflight_timeout = self._positive("preflight_timeout")
        self._flight_action_timeout = self._positive(
            "flight_action_timeout")
        self._flight_health_max_age = self._positive(
            "flight_health_max_age")
        self._view_position = self._vector3("view_position")
        self._view_yaw = self._finite("view_yaw")
        self._observation_timeout = self._positive("observation_timeout")
        self._observation_max_age = self._positive("observation_max_age")

        self._ground_standoff = self._positive("ground_standoff")
        self._ground_goal_tolerance = self._nonnegative(
            "ground_goal_tolerance")
        self._ground_yaw_goal_tolerance = self._positive('ground_yaw_goal_tolerance')
        self._ground_tf_max_age = self._positive("ground_tf_max_age")
        self._navigation_timeout = self._positive("navigation_timeout")
        self._max_ground_travel = self._positive("max_ground_travel")

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
        self._grasp_confirmation_max_age = self._positive(
            "grasp_confirmation_max_age")
        self._grasp_confirmation_timeout = self._positive(
            "grasp_confirmation_timeout")
        self._retention_hold = self._positive("retention_hold")
        self._feasibility = check_target_feasibility(
            self._target_size, self._maximum_gripper_opening,
            self._opening_margin)
        if not self._feasibility.feasible:
            raise DemoError("pick target exceeds the real AG95 opening")

        self._takeoff_command = FlightCommandGoal.TAKEOFF
        self._fly_to_command = FlightCommandGoal.FLY_TO
        self._hover_command = FlightCommandGoal.HOVER
        self._land_command = FlightCommandGoal.LAND
        self._flight_goal_type = FlightCommandGoal
        self._flight_client = actionlib.SimpleActionClient(
            self._flight_action, FlightCommandAction)
        self._navigation_client = actionlib.SimpleActionClient(
            self._ground_navigation_action, MoveBaseAction)
        self._ground_stop = rospy.ServiceProxy(
            self._ground_stop_service, Trigger)
        self._rm4d_client = None
        self._rm4d_request_type = None
        if self._placement_mode == "rm4d":
            from rm4d_sim_integration.srv import (
                PlanBasePlacement,
                PlanBasePlacementRequest,
            )
            self._rm4d_request_type = PlanBasePlacementRequest
            self._rm4d_client = rospy.ServiceProxy(
                self._rm4d_service, PlanBasePlacement)
        self._arm_client = actionlib.SimpleActionClient(
            self._arm_action, FollowJointTrajectoryAction)
        self._gripper_client = actionlib.SimpleActionClient(
            self._gripper_action, FollowJointTrajectoryAction)
        self._cartesian_path = rospy.ServiceProxy(
            "/compute_cartesian_path", GetCartesianPath)
        self._status_pub = rospy.Publisher(
            self._status_topic, String, queue_size=1, latch=True)

        rospy.Subscriber(
            self._uav_state_topic, UAVState, self._uav_state_callback,
            queue_size=10)
        rospy.Subscriber(
            self._air_pose_topic, PoseStamped, self._air_pose_callback,
            queue_size=10)
        rospy.Subscriber(
            self._ground_pose_topic, PoseStamped,
            self._ground_target_pose_callback, queue_size=10)
        rospy.Subscriber('/ground_observer/surface_cue', PointStamped,
                         self._ground_surface_cue_callback, queue_size=1)
        rospy.Subscriber(
            self._joint_state_topic, JointState, self._joint_state_callback,
            queue_size=10)
        rospy.Subscriber(
            self._grasp_confirmed_topic, Bool,
            self._grasp_confirmation_callback, queue_size=10)

        self._move_group = None
        self._planning_scene = None
        self._scene_robot = None
        self._get_scene = None
        self._apply_scene = None
        self._state_validity = None
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
        if (value < 0.0 or (not allow_zero and value == 0.0)
                or value > 1.0):
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

    def _air_pose_callback(self, message):
        with self._lock:
            self._air_pose = message
            self._air_pose_received = time.monotonic()

    def _ground_target_pose_callback(self, message):
        with self._lock:
            self._ground_target_pose = message
            self._ground_pose_received = time.monotonic()

    def _ground_surface_cue_callback(self, message):
        with self._lock:
            self._ground_surface_cue = message

    def _joint_state_callback(self, message):
        with self._lock:
            self._joint_state = message
            self._joint_state_received = time.monotonic()

    def _grasp_confirmation_callback(self, message):
        with self._lock:
            self._grasp_confirmed = bool(message.data)
            self._grasp_confirmation_received = time.monotonic()

    def _air_snapshot(self):
        with self._lock:
            return (
                self._uav_state, self._air_pose,
                self._uav_state_received, self._air_pose_received)

    def _manipulation_snapshot(self):
        with self._lock:
            return (
                self._ground_target_pose, self._ground_pose_received,
                self._joint_state, self._joint_state_received,
                self._grasp_confirmed, self._grasp_confirmation_received)

    def _publish_status(self, state, **details):
        payload = {
            "state": state,
            "ros_time": rospy.Time.now().to_sec(),
            "wall_time": time.time(),
        }
        payload.update(details)
        self._status_pub.publish(String(data=json.dumps(
            payload, sort_keys=True, allow_nan=False)))
        rospy.loginfo(
            "[air_ground_pick_demo] %s %s", state,
            json.dumps(details, sort_keys=True, allow_nan=False))

    @staticmethod
    def _wait_step():
        rospy.rostime.wallsleep(0.05)

    def _wait_preflight(self):
        self._publish_status("PREFLIGHT")
        if not self._flight_client.wait_for_server(
                rospy.Duration(self._preflight_timeout)):
            raise DemoError("flight facade action is unavailable")
        deadline = time.monotonic() + self._preflight_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            state, _pose, received, _pose_received = self._air_snapshot()
            if (state is not None and received is not None and
                    native_state_ready(
                        state.connected, state.odom_valid,
                        time.monotonic() - received,
                        self._flight_health_max_age)):
                return
            self._wait_step()
        raise DemoError("native Prometheus state is not ready")

    def _execute_flight(self, command, label, target=None):
        goal = self._flight_goal_type()
        goal.command = command
        if target is not None:
            goal.target = target
        self._flight_client.send_goal(goal)
        if not self._flight_client.wait_for_result(
                rospy.Duration(self._flight_action_timeout)):
            self._flight_client.cancel_goal()
            raise DemoError("%s action timed out" % label)
        result = self._flight_client.get_result()
        if (self._flight_client.get_state() != GoalStatus.SUCCEEDED or
                result is None or not result.success):
            detail = "" if result is None else result.message
            raise DemoError("%s action failed: %s" % (label, detail))

    @staticmethod
    def _quaternion_from_yaw(yaw):
        return 0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)

    def _view_pose(self):
        target = PoseStamped()
        target.header.frame_id = self._map_frame
        target.header.stamp = rospy.Time.now()
        target.pose.position.x = self._view_position[0]
        target.pose.position.y = self._view_position[1]
        target.pose.position.z = self._view_position[2]
        orientation = self._quaternion_from_yaw(self._view_yaw)
        (target.pose.orientation.x, target.pose.orientation.y,
         target.pose.orientation.z, target.pose.orientation.w) = orientation
        return target

    def _observe_from_air(self):
        started = time.monotonic()
        deadline = started + self._observation_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            _state, pose, _state_at, received = self._air_snapshot()
            if (pose is not None and received is not None and
                    received >= started and observation_is_fresh(
                        pose.header.stamp.to_sec(),
                        rospy.Time.now().to_sec(), self._observation_max_age,
                        pose.header.frame_id, self._map_frame)):
                target = self._target_from_pose(pose)
                self._publish_status(
                    "AIR_HANDOFF", target_map=list(target),
                    observation_stamp=pose.header.stamp.to_sec())
                return target
            self._wait_step()
        raise DemoError("aerial target observation timed out")

    def _run_air_phase(self):
        self._wait_preflight()
        self._publish_status("ARMING")
        self._publish_status("COMMAND_CONTROL")
        self._publish_status("TAKEOFF")
        self._flight_started = True
        self._execute_flight(self._takeoff_command, "takeoff")
        self._publish_status("AIR_VIEW")
        self._execute_flight(
            self._fly_to_command, "fly-to", self._view_pose())
        self._execute_flight(self._hover_command, "hover")
        self._publish_status("AIR_OBSERVE")
        target = self._observe_from_air()
        self._publish_status("LANDING")
        self._request_land()
        return target

    def _request_land(self):
        self._execute_flight(self._land_command, "landing")
        self._landed = True

    @staticmethod
    def _yaw_from_quaternion(rotation):
        values = tuple(float(value) for value in (
            rotation.x, rotation.y, rotation.z, rotation.w))
        norm = math.sqrt(sum(value * value for value in values))
        if not math.isfinite(norm) or norm <= 1e-9:
            raise DemoError("invalid TF quaternion")
        x, y, z, w = (value / norm for value in values)
        return math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z))

    def _ground_pose(self):
        transform = self._tf_buffer.lookup_transform(
            self._map_frame, self._ground_base_frame, rospy.Time(0),
            rospy.Duration(0.5))
        if not transform_is_fresh(
                transform.header.stamp.to_sec(), rospy.Time.now().to_sec(),
                self._ground_tf_max_age):
            raise DemoError("ground base TF is stale")
        translation = transform.transform.translation
        pose = (
            float(translation.x), float(translation.y),
            self._yaw_from_quaternion(transform.transform.rotation))
        if not all(math.isfinite(value) for value in pose):
            raise DemoError("ground base TF is nonfinite")
        return pose

    def _stop_ground(self, required=False):
        try:
            rospy.wait_for_service(
                self._ground_stop_service,
                timeout=self._navigation_timeout if required else 0.5)
            response = self._ground_stop()
            if required and not response.success:
                raise DemoError("ground stop failed: %s" % response.message)
            return bool(response.success)
        except (rospy.ROSException, rospy.ServiceException) as error:
            if required:
                raise DemoError("ground stop service unavailable") from error
            return False

    def _execute_ground_goal(self, goal_geometry, label):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self._map_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = goal_geometry.x
        goal.target_pose.pose.position.y = goal_geometry.y
        orientation = self._quaternion_from_yaw(goal_geometry.yaw)
        (goal.target_pose.pose.orientation.x,
         goal.target_pose.pose.orientation.y,
         goal.target_pose.pose.orientation.z,
         goal.target_pose.pose.orientation.w) = orientation
        self._navigation_client.send_goal(goal)
        if not self._navigation_client.wait_for_result(
                rospy.Duration(self._navigation_timeout)):
            self._navigation_client.cancel_goal()
            raise DemoError(label + " action timed out")
        if self._navigation_client.get_state() != GoalStatus.SUCCEEDED:
            raise DemoError(
                "%s failed: %s" %
                (label, self._navigation_client.get_goal_status_text()))

    def _select_rm4d_candidate(self, target_map):
        generated = generate_top_down_grasp(
            target_map, self._target_size, self._pregrasp_height,
            self._lift_height, self._finger_pad_lower_edge_offset,
            self._contact_overlap, self._surface_clearance)
        request = self._rm4d_request_type()
        request.grasp_tcp = self._pose_message(
            generated.grasp, self._map_frame, rospy.Time.now())
        request.grasp_id = self._rm4d_grasp_id
        request.top_k = self._rm4d_top_k
        try:
            rospy.wait_for_service(
                self._rm4d_service, timeout=self._rm4d_service_timeout)
            response = self._rm4d_client(request)
        except (rospy.ROSException, rospy.ServiceException) as error:
            raise DemoError("RM4D service failed: %s" % error) from error
        if not response.success:
            raise DemoError(
                "RM4D returned %s: %s" %
                (response.status, response.message))
        if response.status == "no_feasible_candidate":
            raise DemoError("RM4D returned no feasible candidate")
        if response.status != "ok":
            raise DemoError("RM4D returned unsupported status %s" %
                            response.status)
        if response.candidates.header.frame_id != self._map_frame:
            raise DemoError("RM4D candidates are not in map")
        if not (
                len(response.candidates.poses) ==
                len(response.candidate_ids) == len(response.scores) and
                response.candidates.poses):
            raise DemoError("RM4D candidate response is incomplete")
        top_pose = response.candidates.poses[0]
        top_id = response.candidate_ids[0]
        top_score = float(response.scores[0])
        candidate = (
            float(top_pose.position.x),
            float(top_pose.position.y),
            self._yaw_from_quaternion(top_pose.orientation),
        )
        if (not top_id or not math.isfinite(top_score) or
                not all(math.isfinite(value) for value in candidate)):
            raise DemoError("RM4D top candidate is invalid")
        return candidate, top_id, top_score, len(response.candidates.poses)

    def _approach_ground_rm4d(self, target_map):
        start = self._ground_pose()
        candidate, candidate_id, score, count = \
            self._select_rm4d_candidate(target_map)
        positioning_goal, final_goal = compute_staged_candidate_goals(
            start, candidate)
        self._publish_status(
            "RM4D_CANDIDATES", candidate_count=count,
            top_candidate_id=candidate_id,
            top_candidate_score=score,
            top_candidate_pose_map=list(candidate))
        self._publish_status(
            "GROUND_APPROACH", start_map=list(start),
            target_map=list(target_map),
            navigation_goal_map=[
                final_goal.x, final_goal.y, final_goal.yaw],
            top_candidate_id=candidate_id,
            top_candidate_score=score)
        if not self._navigation_client.wait_for_server(
                rospy.Duration(self._navigation_timeout)):
            raise DemoError("ground navigation action is unavailable")
        if positioning_goal != final_goal:
            raise DemoError("RM4D candidate geometry changed internally")
        self._execute_ground_goal(final_goal, "RM4D top-1 navigation")
        self._stop_ground(required=True)
        final = self._ground_pose()
        self._publish_status('GROUND_ARRIVAL_MEASURED', actual_pose_map=list(final),
                             goal_pose_map=list(final_goal))
        if not arrival_within_tolerance(final, final_goal, self._ground_goal_tolerance,
                                        self._ground_yaw_goal_tolerance):
            raise DemoError('Ground actual arrival is outside candidate position/yaw tolerance')
        total_travel = travel_distance(start[:2], final[:2])
        if total_travel > self._max_ground_travel:
            raise DemoError("ground travel bound exceeded")
        target_distance = travel_distance(final[:2], target_map[:2])
        self._publish_status(
            "GROUND_STOPPED", ground_travel=total_travel,
            actual_pose_map=list(final),
            target_distance=target_distance,
            navigation_goal_map=[
                final_goal.x, final_goal.y, final_goal.yaw],
            top_candidate_id=candidate_id,
            top_candidate_score=score)

    def _approach_ground_standoff(self, target_map):
        start = self._ground_pose()
        positioning_goal, final_goal = compute_staged_standoff_goals(
            start, target_map[:2], self._ground_standoff)
        self._publish_status(
            "GROUND_APPROACH", start_map=list(start),
            target_map=list(target_map),
            navigation_goal_map=[
                final_goal.x, final_goal.y, final_goal.yaw])
        if not self._navigation_client.wait_for_server(
                rospy.Duration(self._navigation_timeout)):
            raise DemoError("ground navigation action is unavailable")
        if motion_required(
                start[:2], (positioning_goal.x, positioning_goal.y),
                self._ground_goal_tolerance):
            self._execute_ground_goal(
                positioning_goal, "ground positioning")
        heading_start = self._ground_pose()
        heading_goal = compute_heading_goal(
            heading_start, target_map[:2])
        rospy.loginfo(
            "ground heading goal keeps measured position [%.4f, %.4f]",
            heading_goal.x, heading_goal.y)
        self._execute_ground_goal(
            heading_goal, "ground final heading")
        self._stop_ground(required=True)
        final = self._ground_pose()
        total_travel = travel_distance(start[:2], final[:2])
        if total_travel > self._max_ground_travel:
            raise DemoError("ground travel bound exceeded")
        target_distance = travel_distance(final[:2], target_map[:2])
        self._publish_status(
            "GROUND_STOPPED", ground_travel=total_travel,
            target_distance=target_distance)

    def _approach_ground(self, target_map):
        if self._placement_mode == "rm4d":
            return self._approach_ground_rm4d(target_map)
        return self._approach_ground_standoff(target_map)

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
        message, received = snapshot[2:4]
        if (message is None or received is None or
                time.monotonic() - received > maximum_age):
            raise DemoError("ground joint state is stale")
        return self._joint_position_map(message)

    def _robot_state_from_joint_feedback(self, maximum_age=1.0):
        snapshot = self._manipulation_snapshot()
        message, received = snapshot[2:4]
        if (message is None or received is None or
                time.monotonic() - received > maximum_age):
            raise DemoError("ground joint state is stale")
        positions = self._joint_position_map(message)
        if not set(self._observation_joint_names).issubset(positions):
            raise DemoError("ground arm joint state is incomplete")
        if getattr(self, "_full_robot_manipulation", False):
            if (not 0.0 <= time.monotonic() - received <= maximum_age or
                    not transform_is_fresh(message.header.stamp.to_sec(),
                                           rospy.Time.now().to_sec(), maximum_age)):
                raise DemoError("ground joint state is stale")
            scene = self._get_manipulation_scene()
            # Send independent measured joints only; MoveIt recomputes mimic
            # joints, including when a gripper sweep changes its master joint.
            if not set(self._scene_active_joints).issubset(positions):
                raise DemoError("full robot joint state is incomplete")
            measured = JointState()
            measured.header = copy.deepcopy(message.header)
            measured.name = list(self._scene_active_joints)
            measured.position = [positions[name] for name in measured.name]
            return manipulation_scene.planning_start_state(scene.robot_state, measured)
        state = RobotState()
        state.joint_state.header.stamp = rospy.Time.now()
        state.joint_state.name = list(message.name)
        state.joint_state.position = [
            positions[name] for name in message.name]
        return state

    def _max_arm_joint_speed(self):
        message = self._manipulation_snapshot()[2]
        if message is None or len(message.name) != len(message.velocity):
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
                if all(
                        name in positions and
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
            accept_grasp_stall=False, start_from_feedback=False):
        if not client.wait_for_server(rospy.Duration(timeout)):
            raise DemoError(label + " controller action is unavailable")
        goal = self._trajectory_goal(names, positions, duration)
        if start_from_feedback:
            message, received = self._manipulation_snapshot()[2:4]
            if (message is None or received is None or
                    not 0.0 <= time.monotonic() - received <= 1.0):
                raise DemoError(label + " measured start state is stale")
            measured_positions = self._joint_position_map(message)
            if (len(message.velocity) != len(message.name) or
                    any(message.name.count(name) != 1 for name in names)):
                raise DemoError(label + " measured start state is incomplete")
            measured_velocities = dict(zip(message.name, message.velocity))
            start = JointTrajectoryPoint()
            start.positions = [measured_positions[name] for name in names]
            start.velocities = [float(measured_velocities[name]) for name in names]
            if not all(math.isfinite(value) for value in start.velocities):
                raise DemoError(label + " measured start velocity is nonfinite")
            start.time_from_start = rospy.Duration(0.0)
            # A contact-stalled close may leave the controller's desired
            # position at the unreachable closure goal. Seed opening from
            # measured state, retaining the future stamp so t=0 is not dropped.
            goal.trajectory.points.insert(0, start)
        client.send_goal(goal)
        if not client.wait_for_result(rospy.Duration(timeout)):
            client.cancel_goal()
            raise DemoError(label + " controller action timed out")
        state = client.get_state()
        if state == GoalStatus.SUCCEEDED:
            return
        if accept_grasp_stall:
            try:
                joint = self._current_joint_positions()[names[0]]
            except (DemoError, KeyError):
                joint = -math.inf
            if (joint >= self._minimum_gripper_closed_joint and
                    self._grasp_confirmation_current()):
                rospy.loginfo(
                    "AG95 confirmed grasp stall accepted at %.4f rad", joint)
                return
        raise DemoError(
            "%s controller action failed: state=%s status=%s" %
            (label, state, client.get_goal_status_text()))

    def _open_gripper(self):
        if getattr(self, "_full_robot_manipulation", False):
            self._check_gripper_sweep(self._gripper_open_position, "AG95 opening")
        self._execute_trajectory(
            self._gripper_client, ("left_outer_knuckle_joint",),
            (self._gripper_open_position,), self._gripper_motion_time,
            self._gripper_action_timeout, "AG95 open",
            start_from_feedback=True)
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
        if getattr(self, "_full_robot_manipulation", False):
            group = self._initialize_moveit()
            group.set_start_state(self._check_full_robot_state("observation start"))
            group.set_joint_value_target(dict(zip(
                self._observation_joint_names, self._observation_joint_positions)))
            planned = group.plan()
            success = bool(planned[0]) if isinstance(planned, tuple) else True
            trajectory = planned[1] if isinstance(planned, tuple) else planned
            if not success or not trajectory.joint_trajectory.points:
                raise DemoError("collision-aware joint observation planning failed")
            if not group.execute(trajectory, wait=True):
                raise DemoError("collision-aware joint observation execution failed")
            group.stop()
        else:
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
            raise DemoError("target pose is nonfinite")
        return target

    def _validate_near_field_target_height(self, target):
        expected_center_z = 0.5 * self._target_size[2]
        if abs(target[2] - expected_center_z) > \
                self._target_center_height_tolerance:
            raise DemoError(
                "near-field target height is inconsistent: "
                "measured=%.4f expected=%.4f tolerance=%.4f" %
                (target[2], expected_center_z,
                 self._target_center_height_tolerance))

    def _wait_for_ground_target(self, opening, timeout=None):
        started = time.monotonic()
        deadline = started + (self._ground_observation_timeout if timeout is None else timeout)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            pose, received = self._manipulation_snapshot()[:2]
            if (pose is not None and received is not None and
                    received >= started and observation_is_fresh(
                        pose.header.stamp.to_sec(),
                        rospy.Time.now().to_sec(),
                        self._ground_observation_max_age,
                        pose.header.frame_id, self._map_frame)):
                target = self._target_from_pose(pose)
                self._validate_near_field_target_height(target)
                self._publish_status(
                    "GROUND_REFINED", target_map=list(target),
                    observation_stamp=pose.header.stamp.to_sec(),
                    measured_gripper_opening=opening)
                return pose, target
            self._wait_step()
        raise DemoError("near-field D435 observation timed out")

    def _observe_ground_target_rm4d(self, aerial_target):
        if self._ground_observation_mode == 'camera_centered_v1':
            from air_ground_pick_demo.ground_observation import observe_from_camera_poses
            return observe_from_camera_poses(self, aerial_target, DemoError)
        self._publish_status("GROUND_OBSERVE")
        opening = self._open_gripper()
        generated = generate_top_down_grasp(
            aerial_target, self._target_size, self._pregrasp_height,
            self._lift_height, self._finger_pad_lower_edge_offset,
            self._contact_overlap, self._surface_clearance)
        group = self._initialize_moveit()
        exact_pregrasp = self._pose_message(
            generated.pregrasp, self._map_frame, rospy.Time.now())
        exact_grasp = self._pose_message(
            generated.grasp, self._map_frame, exact_pregrasp.header.stamp)
        target_facing_pregrasp = self._transform_pose(
            exact_pregrasp, group.get_planning_frame())
        target_facing_grasp = self._transform_pose(
            exact_grasp, group.get_planning_frame())
        self._execute_pregrasp(
            target_facing_pregrasp, target_facing_grasp)
        return self._wait_for_ground_target(opening)

    def _observe_ground_target_standoff(self, _aerial_target):
        self._publish_status("GROUND_OBSERVE")
        opening = self._open_gripper()
        self._move_to_ground_observation()
        return self._wait_for_ground_target(opening)

    def _observe_ground_target(self, aerial_target):
        if getattr(self, "_full_robot_manipulation", False):
            # This tuple is the accepted aerial estimate, not the partial
            # surface cue. The timestamp here is the planning reference time.
            accepted = PoseStamped()
            accepted.header.frame_id = self._map_frame
            accepted.header.stamp = rospy.Time.now()
            accepted.pose.position.x, accepted.pose.position.y, accepted.pose.position.z = aerial_target[:3]
            (accepted.pose.orientation.x, accepted.pose.orientation.y,
             accepted.pose.orientation.z, accepted.pose.orientation.w) = self._quaternion_from_yaw(aerial_target[3])
            self._update_manipulation_target(accepted, source="accepted_aerial")
        if self._placement_mode == "rm4d":
            return self._observe_ground_target_rm4d(aerial_target)
        return self._observe_ground_target_standoff(aerial_target)

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
        floor.header.frame_id = self._map_frame
        floor.header.stamp = rospy.Time(0)
        floor.pose.orientation.w = 1.0
        floor = self._transform_pose(
            floor, group.get_planning_frame(), use_latest=True)
        self._planning_scene.add_plane("air_ground_floor", floor)
        self._move_group = group
        return group

    def _get_manipulation_scene(self):
        if self._get_scene is None:
            self._scene_robot = moveit_commander.RobotCommander()
            self._scene_robot_links = tuple(self._scene_robot.get_link_names())
            self._scene_active_joints = tuple(self._scene_robot.get_active_joint_names())
            try:
                for name in ("/get_planning_scene", "/apply_planning_scene", "/check_state_validity"):
                    rospy.wait_for_service(name, timeout=self._moveit_server_timeout)
                self._get_scene = rospy.ServiceProxy("/get_planning_scene", GetPlanningScene)
                self._apply_scene = rospy.ServiceProxy("/apply_planning_scene", ApplyPlanningScene)
                self._state_validity = rospy.ServiceProxy("/check_state_validity", GetStateValidity)
            except (rospy.ROSException, rospy.ServiceException) as error:
                raise DemoError("full manipulation scene services unavailable: %s" % error) from error
        request = GetPlanningSceneRequest()
        request.components.components = (
            PlanningSceneComponents.SCENE_SETTINGS | PlanningSceneComponents.ROBOT_STATE |
            PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS |
            PlanningSceneComponents.WORLD_OBJECT_NAMES | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
            PlanningSceneComponents.TRANSFORMS | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX |
            PlanningSceneComponents.OCTOMAP | PlanningSceneComponents.LINK_PADDING_AND_SCALING |
            PlanningSceneComponents.OBJECT_COLORS)
        try:
            return self._get_scene(request).scene
        except (rospy.ROSException, rospy.ServiceException) as error:
            raise DemoError("full manipulation scene read failed: %s" % error) from error

    def _apply_manipulation_scene(self, change):
        try:
            response = self._apply_scene(ApplyPlanningSceneRequest(scene=change))
        except (rospy.ROSException, rospy.ServiceException) as error:
            raise DemoError("manipulation scene apply failed: %s" % error) from error
        if not response.success:
            raise DemoError("manipulation scene apply was rejected")

    def _update_manipulation_target(self, accepted_pose, source="accepted"):
        group = self._initialize_moveit()
        planning_frame = group.get_planning_frame()
        if accepted_pose.header.frame_id == planning_frame:
            planning_pose = copy.deepcopy(accepted_pose)
        else:
            # MoveIt startup can delay Python TF callbacks. A latest lookup's
            # timeout checks availability only, so explicitly wait for freshness
            # within the existing lookup budget, even when simulated time stops.
            deadline = time.monotonic() + .5
            reason = 'unavailable'
            while not rospy.is_shutdown():
                if time.monotonic() >= deadline:
                    raise DemoError("target planning-frame transform is %s" % reason)
                try:
                    transform = self._tf_buffer.lookup_transform(
                        planning_frame, accepted_pose.header.frame_id,
                        rospy.Time(0), rospy.Duration(0.0))
                    if time.monotonic() < deadline and transform_is_fresh(
                            transform.header.stamp.to_sec(), rospy.Time.now().to_sec(),
                            self._ground_tf_max_age):
                        break
                    reason = "stale"
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException) as error:
                    reason = "unavailable: %s" % error
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise DemoError("target planning-frame transform is %s" % reason)
                rospy.rostime.wallsleep(min(.05, remaining))
            else:
                raise DemoError("ROS shutdown while waiting for target planning-frame transform")
            planning_pose = do_transform_pose(accepted_pose, transform)
        scene = self._get_manipulation_scene()
        try:
            change = manipulation_scene.world_target_diff(
                scene, planning_pose, self._target_size, self._scene_robot_links)
        except manipulation_scene.SceneError as error:
            raise DemoError("perceived target scene update failed: %s" % error) from error
        self._apply_manipulation_scene(change)
        p, q = planning_pose.pose.position, planning_pose.pose.orientation
        self._publish_status(
            "GROUND_MANIPULATION_SCENE", source=source, frame=planning_pose.header.frame_id,
            target_pose=[p.x, p.y, p.z, q.x, q.y, q.z, q.w], target_size=list(self._target_size),
            object_id=manipulation_scene.TARGET_ID, target_contacts_allowed=False)

    def _set_manipulation_contact(self, enabled):
        scene = self._get_manipulation_scene()
        try:
            change = manipulation_scene.finger_contact_diff(scene, self._scene_robot_links, enabled)
        except manipulation_scene.SceneError as error:
            raise DemoError("grasp contact scene update failed: %s" % error) from error
        self._apply_manipulation_scene(change)

    def _check_full_robot_state(self, label, state=None, require_payload=False):
        state = self._robot_state_from_joint_feedback() if state is None else state
        if require_payload and not any(
                item.object.id == manipulation_scene.TARGET_ID for item in state.attached_collision_objects):
            raise DemoError(label + " perceived payload is missing from robot state")
        request = GetStateValidityRequest()
        request.robot_state = state
        request.group_name = ""  # Check the entire robot, not just manipulator links.
        try:
            response = self._state_validity(request)
        except (rospy.ROSException, rospy.ServiceException) as error:
            raise DemoError(label + " full robot state validity unavailable: " + str(error)) from error
        if not response.valid:
            contacts = ["%s/%s" % (c.contact_body_1, c.contact_body_2) for c in response.contacts]
            raise DemoError("%s full robot state invalid; collisions=%s" % (label, contacts))
        return state

    def _check_gripper_sweep(self, final_joint, label):
        state = self._robot_state_from_joint_feedback()
        names = state.joint_state.name
        index = names.index("left_outer_knuckle_joint")
        start = state.joint_state.position[index]
        conservative_jaw_opening(start, self._maximum_gripper_opening, self._maximum_gripper_joint)
        conservative_jaw_opening(final_joint, self._maximum_gripper_opening, self._maximum_gripper_joint)
        # Two 55-mm linkage lengths give a conservative 110-mm sweep radius.
        # Bound sample displacement using the existing Cartesian spatial step;
        # these are discrete full-model checks, not a continuous-motion proof.
        intervals = max(1, int(math.ceil(abs(final_joint-start) * .110 / self._cartesian_eef_step)))
        for index_in_sweep in range(intervals + 1):
            sample = copy.deepcopy(state)
            sample.joint_state.position[index] = start + (final_joint-start) * index_in_sweep/intervals
            self._check_full_robot_state(label, sample)
        return intervals + 1

    def _preshape_gripper(self):
        try:
            preshape = manipulation_scene.required_opening_preshape(
                self._target_size, self._maximum_gripper_opening, self._maximum_gripper_joint,
                self._opening_margin, self._gripper_joint_tolerance)
        except manipulation_scene.SceneError as error:
            raise DemoError("target-sized gripper preshape is invalid: %s" % error) from error
        self._check_gripper_sweep(preshape.command_position, "AG95 preshape")
        self._execute_trajectory(
            self._gripper_client, ("left_outer_knuckle_joint",), (preshape.command_position,),
            self._gripper_motion_time, self._gripper_action_timeout, "AG95 preshape",
            start_from_feedback=True)
        self._wait_joint_target(
            ("left_outer_knuckle_joint",), (preshape.command_position,), self._gripper_joint_tolerance,
            self._gripper_action_timeout, "AG95 preshape")
        measured = self._check_full_robot_state("AG95 measured preshape")
        positions = dict(zip(measured.joint_state.name, measured.joint_state.position))
        opening = conservative_jaw_opening(positions["left_outer_knuckle_joint"],
                                           self._maximum_gripper_opening, self._maximum_gripper_joint)
        if opening <= self._feasibility.required_opening:
            raise DemoError("measured AG95 opening is too small for target")
        self._publish_status(
            "GROUND_GRIPPER_PRESHAPE", commanded_joint=preshape.command_position,
            measured_joint=positions["left_outer_knuckle_joint"], measured_opening=opening,
            required_opening=preshape.required_opening, tracking_tolerance=self._gripper_joint_tolerance)
        return opening

    def _attach_manipulation_target(self):
        frame = self._move_group.get_planning_frame()
        transform = self._tf_buffer.lookup_transform(
            frame, self._end_effector_link, rospy.Time(0), rospy.Duration(.5))
        if not transform_is_fresh(transform.header.stamp.to_sec(), rospy.Time.now().to_sec(),
                                  self._ground_tf_max_age):
            raise DemoError("confirmed grasp TCP transform is stale")
        measured = PoseStamped()
        measured.header = copy.deepcopy(transform.header)
        p = transform.transform.translation
        measured.pose.position.x, measured.pose.position.y, measured.pose.position.z = p.x, p.y, p.z
        measured.pose.orientation = copy.deepcopy(transform.transform.rotation)
        scene = self._get_manipulation_scene()
        try:
            change = manipulation_scene.attach_target_diff(
                scene, measured, self._end_effector_link, self._scene_robot_links,
                self._grasp_confirmation_current())
        except manipulation_scene.SceneError as error:
            raise DemoError("confirmed grasp scene update failed: %s" % error) from error
        self._apply_manipulation_scene(change)
        self._check_full_robot_state("confirmed grasp", require_payload=True)
        self._publish_status(
            "GROUND_PAYLOAD_MODELED", object_id=manipulation_scene.TARGET_ID,
            link=self._end_effector_link, frame=frame, measurement_stamp=measured.header.stamp.to_sec(),
            measured_tcp=[p.x, p.y, p.z, measured.pose.orientation.x, measured.pose.orientation.y,
                          measured.pose.orientation.z, measured.pose.orientation.w],
            touch_links=list(manipulation_scene.FINGER_LINKS))

    @staticmethod
    def _quaternion_error(first, second):
        first_values = (first.x, first.y, first.z, first.w)
        second_values = (second.x, second.y, second.z, second.w)
        first_norm = math.sqrt(sum(value * value for value in first_values))
        second_norm = math.sqrt(sum(value * value for value in second_values))
        if first_norm <= 1e-12 or second_norm <= 1e-12:
            return math.inf
        dot = abs(sum(
            a * b for a, b in zip(first_values, second_values)) /
            (first_norm * second_norm))
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

    def _continuation_from_plan(self, trajectory, continuation):
        joint_trajectory = trajectory.joint_trajectory
        if not joint_trajectory.joint_names or not joint_trajectory.points:
            return None
        request = GetCartesianPathRequest()
        request.header = continuation.header
        request.start_state.joint_state.name = list(
            joint_trajectory.joint_names)
        request.start_state.joint_state.position = list(
            joint_trajectory.points[-1].positions)
        request.start_state.is_diff = True
        if getattr(self, "_full_robot_manipulation", False):
            current = self._robot_state_from_joint_feedback()
            endpoint = dict(zip(joint_trajectory.joint_names, joint_trajectory.points[-1].positions))
            current.joint_state.position = [endpoint.get(name, position) for name, position in
                                            zip(current.joint_state.name, current.joint_state.position)]
            request.start_state = current
        request.group_name = self._move_group_name
        request.link_name = self._end_effector_link
        request.waypoints = [continuation.pose]
        request.max_step = self._cartesian_eef_step
        request.jump_threshold = 0.0
        request.avoid_collisions = True
        return self._cartesian_path(request)

    def _execute_pregrasp(self, target, continuation=None):
        group = self._move_group
        attempts = self._rm4d_pregrasp_plan_attempts \
            if continuation is not None else 1
        trajectory = None
        if continuation is not None:
            rospy.wait_for_service(
                "/compute_cartesian_path", timeout=self._moveit_server_timeout)
        for _attempt in range(attempts):
            if getattr(self, "_full_robot_manipulation", False):
                group.set_start_state(self._robot_state_from_joint_feedback())
            else:
                group.set_start_state_to_current_state()
            group.set_pose_target(target, self._end_effector_link)
            planned = group.plan()
            success = bool(planned[0]) if isinstance(planned, tuple) else True
            candidate = planned[1] if isinstance(planned, tuple) else planned
            group.clear_pose_targets()
            if not success or not candidate.joint_trajectory.points:
                continue
            if continuation is not None:
                response = self._continuation_from_plan(
                    candidate, continuation)
                if (response is None or
                        not math.isfinite(response.fraction) or
                        response.fraction < self._cartesian_min_fraction or
                        not response.solution.joint_trajectory.points):
                    continue
            trajectory = candidate
            break
        if trajectory is None:
            detail = " with no continuation-valid IK branch" \
                if continuation is not None else ""
            raise DemoError("MoveIt pregrasp planning failed" + detail)
        if not group.execute(trajectory, wait=True):
            group.clear_pose_targets()
            raise DemoError("MoveIt pregrasp execution failed")
        group.stop()
        group.clear_pose_targets()
        return self._verify_tcp_pose(target, "pregrasp")

    def _execute_cartesian(self, target, label):
        group = self._move_group
        if getattr(self, "_full_robot_manipulation", False):
            start = self._check_full_robot_state(label + " start", require_payload=label == "lift")
            group.set_start_state(start)
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

    def _grasp_confirmation_current(self, now=None):
        snapshot = self._manipulation_snapshot()
        confirmed, received = snapshot[4:6]
        current = time.monotonic() if now is None else now
        return bool(
            confirmed and received is not None and current >= received and
            current - received <= self._grasp_confirmation_max_age)

    def _wait_grasp_confirmation(self):
        deadline = time.monotonic() + self._grasp_confirmation_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self._grasp_confirmation_current():
                return
            self._wait_step()
        raise DemoError("AG95 grasp was not confirmed")

    def _hold_grasp_confirmation(self):
        deadline = time.monotonic() + self._retention_hold
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if not self._grasp_confirmation_current():
                raise DemoError("AG95 lost grasp confirmation during lift")
            self._wait_step()

    def _close_gripper(self):
        full = getattr(self, "_full_robot_manipulation", False)
        try:
            if full:
                self._set_manipulation_contact(True)
                # Validate up to physical pad contact, not unreachable free
                # closure at the controller's unchanged 0.70-rad force goal.
                geometry = manipulation_scene.ag95_contact_geometry(
                    self._target_size, self._maximum_gripper_opening,
                    self._maximum_gripper_joint, self._finger_pad_lower_edge_offset)
                contact_joint = geometry.q_contact
                checked_states = self._check_gripper_sweep(contact_joint, "AG95 physical closure")
                self._publish_status(
                    "GROUND_GRASP_GEOMETRY", contact_joint=contact_joint, checked_states=checked_states,
                    commanded_closed_joint=self._gripper_closed_position, continuous_clearance_proven=False)
            self._execute_trajectory(
                self._gripper_client, ("left_outer_knuckle_joint",),
                (self._gripper_closed_position,), self._gripper_motion_time,
                self._gripper_action_timeout, "AG95 close",
                accept_grasp_stall=True)
            self._wait_grasp_confirmation()
            positions = self._current_joint_positions()
            if positions.get("left_outer_knuckle_joint", -math.inf) < \
                    self._minimum_gripper_closed_joint:
                raise DemoError("AG95 did not close around the target")
            if full:
                self._attach_manipulation_target()
        except Exception as error:
            if full:
                try:
                    scene = self._get_manipulation_scene()
                    # Never remove a retained payload after a later error. Only a
                    # still-world target can have abandoned-grasp exemptions reset.
                    if (any(o.id == manipulation_scene.TARGET_ID for o in scene.world.collision_objects) and
                            not any(o.object.id == manipulation_scene.TARGET_ID
                                    for o in scene.robot_state.attached_collision_objects)):
                        self._set_manipulation_contact(False)
                except Exception as cleanup_error:
                    raise DemoError("%s; grasp contact cleanup failed: %s" % (error, cleanup_error)) from error
            raise

    def _generate_ground_grasp(self, target):
        """Ground refinement only; aerial RM4D queries retain legacy geometry."""
        pad_edge = self._finger_pad_lower_edge_offset
        if getattr(self, "_full_robot_manipulation", False):
            try:
                pad_edge = manipulation_scene.ag95_contact_geometry(
                    self._target_size, self._maximum_gripper_opening,
                    self._maximum_gripper_joint, pad_edge).pad_edge
            except manipulation_scene.SceneError as error:
                raise DemoError("Ground contact geometry is invalid: %s" % error) from error
        return generate_top_down_grasp(
            target, self._target_size, self._pregrasp_height,
            self._lift_height, pad_edge,
            self._contact_overlap, self._surface_clearance)

    def _pick_and_lift(self, sensor_pose, target):
        generated = self._generate_ground_grasp(target)
        group = self._initialize_moveit()
        planning_frame = group.get_planning_frame()
        messages = tuple(
            self._transform_pose(
                self._pose_message(
                    pose, self._map_frame, sensor_pose.header.stamp),
                planning_frame)
            for pose in (generated.pregrasp, generated.grasp, generated.lift))
        pregrasp, grasp, lift = messages
        if getattr(self, "_full_robot_manipulation", False):
            geometry = manipulation_scene.ag95_contact_geometry(
                self._target_size, self._maximum_gripper_opening,
                self._maximum_gripper_joint, self._finger_pad_lower_edge_offset)
            self._publish_status(
                "GROUND_GRASP_CALIBRATION", contact_joint=geometry.q_contact,
                open_pad_edge=self._finger_pad_lower_edge_offset, contact_pad_edge=geometry.pad_edge,
                pad_down_shift=geometry.delta, contact_overlap=self._contact_overlap,
                grasp_tcp_map_z=generated.grasp.position[2])
            self._update_manipulation_target(sensor_pose, source="accepted_refined")
            self._preshape_gripper()
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
        self._hold_grasp_confirmation()
        self._publish_status(
            "LIFT", tcp_lift=measured_lift, grasp_confirmed=True,
            target_map=list(target))

    def _safe_land(self):
        if not self._flight_started or self._landed:
            return
        try:
            self._execute_flight(self._hover_command, "recovery hover")
        except DemoError as error:
            rospy.logwarn("recovery hover failed: %s", error)
        try:
            self._request_land()
        except DemoError as error:
            rospy.logerr("landing through flight facade failed: %s", error)

    def run(self):
        try:
            target_map = self._run_air_phase()
            self._approach_ground(target_map)
            sensor_pose, refined_target = self._observe_ground_target(
                target_map)
            self._pick_and_lift(sensor_pose, refined_target)
            return True
        except (DemoError, ApproachError, GraspError,
                moveit_commander.MoveItCommanderException,
                tf2_ros.TransformException) as error:
            rospy.logerr("air-ground pick demo failed: %s", str(error))
            self._publish_status("FAILED", reason=str(error))
            return False
        finally:
            self._stop_ground(required=False)
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
