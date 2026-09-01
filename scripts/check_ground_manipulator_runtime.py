#!/usr/bin/env python3
"""Exercise the ground manipulator through its public ROS interfaces."""

import argparse
from collections import deque
import json
import math
from pathlib import Path
import sys
import threading
import time


SCRIPT_DIRECTORY = str(Path(__file__).resolve().parent)
if SCRIPT_DIRECTORY not in sys.path:
    sys.path.insert(0, SCRIPT_DIRECTORY)
import check_air_ground_runtime as air_ground
import check_bunker_runtime as bunker


RuntimeCheckError = bunker.RuntimeCheckError

MODEL_NAME = "ground_robot"
ARM_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
GRIPPER_JOINT = "left_outer_knuckle_joint"
REQUIRED_CONTROLLERS = (
    "joint_state_controller",
    "arm_controller",
    "gripper_controller",
)
GROUND_TF_FRAMES = (
    "ground/base_link",
    "ground/lidar_2d_link",
    "ground/aubo_i5_base_link",
    "ground/ee_link",
    "ground/d435_color_optical_frame",
    "ground/d435_depth_optical_frame",
    "ground/gripper_tcp_link",
    "ground/left_finger_pad",
    "ground/right_finger_pad",
)


def _finite(values):
    return all(math.isfinite(float(value)) for value in values)


def joint_state_summary(message, now, maximum_age=1.0):
    stamp = message.header.stamp.to_sec()
    age = now - stamp
    names = tuple(message.name)
    positions = tuple(message.position)
    if (not math.isfinite(stamp) or stamp <= 0.0 or
            age < -0.1 or age > maximum_age):
        raise RuntimeCheckError(
            "joint state is stale: stamp=%.6f now=%.6f age=%.3fs" %
            (stamp, now, age))
    if len(names) != len(positions) or len(names) != len(set(names)):
        raise RuntimeCheckError("joint state names and positions are invalid")
    if not _finite(positions):
        raise RuntimeCheckError("joint state contains non-finite positions")
    for label, values in (
            ("velocity", tuple(message.velocity)),
            ("effort", tuple(message.effort))):
        if values and (len(values) != len(names) or not _finite(values)):
            raise RuntimeCheckError(
                "joint state contains invalid %s values" % label)
    position_map = dict(zip(names, positions))
    required = set(ARM_JOINTS + (GRIPPER_JOINT,))
    missing = sorted(required.difference(position_map))
    if missing:
        raise RuntimeCheckError(
            "joint state is missing: %s" % ", ".join(missing))
    return {
        "stamp": stamp,
        "age_s": age,
        "positions": {
            name: float(position_map[name]) for name in sorted(required)},
    }


def controller_summary(controllers):
    by_name = {controller.name: controller for controller in controllers}
    missing = sorted(set(REQUIRED_CONTROLLERS).difference(by_name))
    stopped = sorted(
        name for name in REQUIRED_CONTROLLERS
        if name in by_name and by_name[name].state != "running")
    if missing or stopped:
        raise RuntimeCheckError(
            "ground controllers invalid: missing=%s not_running=%s" %
            (missing, stopped))
    return {
        "running": list(REQUIRED_CONTROLLERS),
        "types": {
            name: by_name[name].type for name in REQUIRED_CONTROLLERS},
    }


def bounded_joint_target(current, delta, lower, upper, max_step):
    values = (current, delta, lower, upper, max_step)
    if not _finite(values) or lower >= upper or max_step <= 0.0:
        raise RuntimeCheckError("joint target bounds are invalid")
    if abs(delta) > max_step + 1e-9:
        raise RuntimeCheckError(
            "joint target step %.3f exceeds %.3f" % (abs(delta), max_step))
    target = current + delta
    if target < lower or target > upper:
        raise RuntimeCheckError(
            "joint target %.3f is outside [%.3f, %.3f]" %
            (target, lower, upper))
    return target


def positions_within(actual, target, tolerance):
    if not math.isfinite(tolerance) or tolerance < 0.0:
        return False
    return all(
        name in actual and
        math.isfinite(float(actual[name])) and
        math.isfinite(float(value)) and
        abs(float(actual[name]) - float(value)) <= tolerance
        for name, value in target.items())


def trajectory_is_bounded(trajectory, start, maximum_excursion):
    names = tuple(trajectory.joint_trajectory.joint_names)
    points = tuple(trajectory.joint_trajectory.points)
    if not names or not points or any(name not in start for name in names):
        return False
    for point in points:
        if len(point.positions) != len(names) or not _finite(point.positions):
            return False
        if any(abs(float(value) - float(start[name])) > maximum_excursion
               for name, value in zip(names, point.positions)):
            return False
    return True


def wait_for_current_sensors(
        rospy, contracts, timeout, buffer_bytes=16 * 1024 * 1024):
    messages = {
        label: deque(maxlen=2)
        for label, _topic, _message_type, _validator in contracts
    }
    messages_lock = threading.Lock()

    def receiver(label):
        def receive(message):
            with messages_lock:
                messages[label].append(message)
        return receive

    subscriptions = [
        rospy.Subscriber(
            topic, message_type, receiver(label), queue_size=1,
            buff_size=buffer_bytes, tcp_nodelay=True)
        for label, topic, message_type, _validator in contracts
    ]
    deadline = time.monotonic() + timeout
    validators = {
        label: validator
        for label, _topic, _message_type, validator in contracts
    }
    topics = {
        label: topic
        for label, topic, _message_type, _validator in contracts
    }
    observed = {}
    summaries = {}
    last_errors = {}
    try:
        while time.monotonic() < deadline:
            with messages_lock:
                current = {
                    label: tuple(values)
                    for label, values in messages.items()
                }
            for label, values in current.items():
                if label in summaries or len(values) != 2:
                    continue
                if not bunker.stamp_advanced(
                        values[0].header.stamp.to_sec(),
                        values[1].header.stamp.to_sec()):
                    continue
                try:
                    summaries[label] = validators[label](values[1])
                    observed[label] = values[1]
                except RuntimeCheckError as error:
                    last_errors[label] = error
            if len(summaries) == len(contracts):
                return observed, summaries
            if rospy.is_shutdown():
                break
            time.sleep(0.05)
    finally:
        for subscription in subscriptions:
            subscription.unregister()
    missing = []
    for label in messages:
        detail = ""
        if label in last_errors:
            detail = ": %s" % last_errors[label]
        missing.append("%s%s" % (topics[label], detail))
    raise RuntimeCheckError(
        "current advancing sensor data missing from %s" %
        ", ".join(missing))


def wait_for_current_sensor(
        rospy, topic, message_type, validator, timeout,
        buffer_bytes=16 * 1024 * 1024):
    messages, summaries = wait_for_current_sensors(
        rospy, (("sensor", topic, message_type, validator),),
        timeout, buffer_bytes=buffer_bytes)
    return messages["sensor"], summaries["sensor"]


def image_summary(message, expected_frame, now):
    summary = air_ground.sensor_header_summary(
        message, expected_frame, now)
    if (message.width <= 0 or message.height <= 0 or message.step <= 0 or
            not message.encoding or not message.data):
        raise RuntimeCheckError("camera image is empty")
    summary.update({
        "width": int(message.width),
        "height": int(message.height),
        "encoding": message.encoding,
    })
    return summary


def point_cloud_summary(message, expected_frame, now):
    summary = air_ground.sensor_header_summary(
        message, expected_frame, now)
    if (message.width <= 0 or message.height <= 0 or
            message.point_step <= 0 or message.row_step <= 0 or
            not message.fields or not message.data):
        raise RuntimeCheckError("D435 point cloud is empty")
    summary.update({
        "width": int(message.width),
        "height": int(message.height),
        "points": int(message.width * message.height),
    })
    return summary


def ground_scan_summary(message, now):
    summary = air_ground.sensor_header_summary(
        message, "ground/lidar_2d_link", now)
    if len(message.ranges) != 720:
        raise RuntimeCheckError("/ground/scan sample count is invalid")
    finite = [
        float(value) for value in message.ranges if math.isfinite(value)]
    if not finite or not all(
            message.range_min <= value <= message.range_max
            for value in finite):
        raise RuntimeCheckError("/ground/scan contains invalid ranges")
    forward = sorted(
        float(value) for index, value in enumerate(message.ranges)
        if math.isfinite(value) and abs(
            message.angle_min + index * message.angle_increment) <= 0.08)
    if len(forward) < 5:
        raise RuntimeCheckError("/ground/scan has no usable forward sector")
    midpoint = len(forward) // 2
    forward_range = (
        forward[midpoint] if len(forward) % 2 else
        0.5 * (forward[midpoint - 1] + forward[midpoint]))
    if not 2.0 <= forward_range <= 2.6:
        raise RuntimeCheckError(
            "/ground/scan forward landmark is %.3f m" % forward_range)
    summary.update({
        "samples": len(message.ranges),
        "finite_samples": len(finite),
        "nearest_range_m": min(finite),
        "forward_range_m": forward_range,
    })
    return summary


def check_ground_scan(rospy, laser_scan_type, timeout):
    _message, summary = wait_for_current_sensor(
        rospy, "/ground/scan", laser_scan_type,
        lambda item: ground_scan_summary(
            item, rospy.Time.now().to_sec()),
        timeout, buffer_bytes=1024 * 1024)
    return summary


def check_model(rospy, model_states_type, timeout):
    message = air_ground.wait_for_condition(
        rospy, "/gazebo/model_states", model_states_type,
        lambda item: MODEL_NAME in item.name, timeout,
        "Gazebo model %s" % MODEL_NAME)
    return MODEL_NAME in message.name


def check_runtime_ready(rospy, bool_type, timeout):
    message = air_ground.wait_for_condition(
        rospy, "/ground/runtime_ready", bool_type,
        lambda item: bool(item.data), timeout, "ground runtime readiness")
    return bool(message.data)


def check_controllers(rospy, list_controllers_type, timeout):
    service_name = "/ground/controller_manager/list_controllers"
    try:
        rospy.wait_for_service(service_name, timeout=timeout)
        response = rospy.ServiceProxy(
            service_name, list_controllers_type)()
    except Exception as error:
        raise RuntimeCheckError(
            "could not list ground controllers: %s" % error)
    return controller_summary(response.controller)


def current_joint_state(rospy, joint_state_type, timeout):
    return air_ground.wait_for_condition(
        rospy, "/ground/joint_states", joint_state_type,
        lambda item: bool(joint_state_summary(
            item, rospy.Time.now().to_sec())),
        timeout, "current ground joint state")


def execute_joint_target(
        rospy, actionlib, action_type, goal_type, point_type, goal_status,
        controller, joint_names, positions, duration, timeout):
    client = actionlib.SimpleActionClient(
        "/ground/%s/follow_joint_trajectory" % controller, action_type)
    if not client.wait_for_server(rospy.Duration(timeout)):
        raise RuntimeCheckError(
            "%s action server is unavailable" % controller)
    goal = goal_type()
    goal.trajectory.joint_names = list(joint_names)
    point = point_type()
    point.positions = list(positions)
    point.velocities = [0.0] * len(positions)
    point.time_from_start = rospy.Duration(duration)
    goal.trajectory.points = [point]
    client.send_goal(goal)
    if not client.wait_for_result(rospy.Duration(timeout)):
        client.cancel_goal()
        raise RuntimeCheckError("%s action timed out" % controller)
    if client.get_state() != goal_status.SUCCEEDED:
        raise RuntimeCheckError(
            "%s action failed: state=%s status=%s" %
            (controller, client.get_state(), client.get_goal_status_text()))


def check_controller_motion(
        rospy, actionlib, joint_state_type, action_type, goal_type,
        point_type, goal_status, timeout):
    before_message = current_joint_state(rospy, joint_state_type, timeout)
    before = joint_state_summary(
        before_message, rospy.Time.now().to_sec())["positions"]

    arm_target = {name: before[name] for name in ARM_JOINTS}
    arm_target["shoulder_pan_joint"] = bounded_joint_target(
        before["shoulder_pan_joint"], 0.10, -3.04, 3.04, 0.15)
    execute_joint_target(
        rospy, actionlib, action_type, goal_type, point_type, goal_status,
        "arm_controller", ARM_JOINTS,
        [arm_target[name] for name in ARM_JOINTS], 2.0, timeout)

    gripper_target = {
        GRIPPER_JOINT: bounded_joint_target(
            before[GRIPPER_JOINT], 0.18, 0.0, 0.93, 0.25),
    }
    execute_joint_target(
        rospy, actionlib, action_type, goal_type, point_type, goal_status,
        "gripper_controller", (GRIPPER_JOINT,),
        (gripper_target[GRIPPER_JOINT],), 2.0, timeout)

    combined_target = dict(arm_target)
    combined_target.update(gripper_target)
    after_message = air_ground.wait_for_condition(
        rospy, "/ground/joint_states", joint_state_type,
        lambda item: positions_within(
            joint_state_summary(
                item, rospy.Time.now().to_sec())["positions"],
            combined_target, 0.08),
        timeout, "controller-driven joint targets")
    after = joint_state_summary(
        after_message, rospy.Time.now().to_sec())["positions"]
    return {
        "shoulder_pan_delta_rad":
            after["shoulder_pan_joint"] - before["shoulder_pan_joint"],
        "gripper_delta_rad":
            after[GRIPPER_JOINT] - before[GRIPPER_JOINT],
        "target_tolerance_rad": 0.08,
    }


def check_sensors(
        rospy, image_type, camera_info_type, point_cloud_type, timeout):
    contracts = (
        ("color", "/ground/d435/color/image_raw", image_type,
         lambda message: image_summary(
             message, "ground/d435_color_optical_frame",
             rospy.Time.now().to_sec())),
        ("color_info", "/ground/d435/color/camera_info", camera_info_type,
         lambda message: air_ground.camera_info_summary(
             message, "ground/d435_color_optical_frame",
             rospy.Time.now().to_sec())),
        ("depth", "/ground/d435/depth/image_raw", image_type,
         lambda message: image_summary(
             message, "ground/d435_depth_optical_frame",
             rospy.Time.now().to_sec())),
        ("depth_info", "/ground/d435/depth/camera_info", camera_info_type,
         lambda message: air_ground.camera_info_summary(
             message, "ground/d435_depth_optical_frame",
             rospy.Time.now().to_sec())),
        ("points", "/ground/d435/depth/points", point_cloud_type,
         lambda message: point_cloud_summary(
             message, "ground/d435_depth_optical_frame",
             rospy.Time.now().to_sec())),
    )
    _messages, summaries = wait_for_current_sensors(
        rospy, contracts, timeout)
    for image_label, info_label in (
            ("color", "color_info"), ("depth", "depth_info")):
        image_size = (
            summaries[image_label]["width"],
            summaries[image_label]["height"])
        info_size = (
            summaries[info_label]["width"],
            summaries[info_label]["height"])
        if image_size != info_size:
            raise RuntimeCheckError(
                "%s image and calibration dimensions differ" % image_label)
    return summaries


def check_moveit(
        rospy, moveit_commander, robot_state_type, joint_state_type, timeout):
    start_message = current_joint_state(rospy, joint_state_type, timeout)
    start = joint_state_summary(
        start_message, rospy.Time.now().to_sec())["positions"]
    group = moveit_commander.MoveGroupCommander(
        "manipulator", wait_for_servers=timeout)
    active_joints = tuple(group.get_active_joints())
    if set(active_joints) != set(ARM_JOINTS):
        raise RuntimeCheckError(
            "MoveIt manipulator joints are invalid: %s" %
            (active_joints,))
    target = {name: start[name] for name in active_joints}
    target["elbow_joint"] = bounded_joint_target(
        start["elbow_joint"], -0.12, -3.04, 3.04, 0.15)

    robot_state = robot_state_type()
    robot_state.joint_state = start_message
    group.set_start_state(robot_state)
    group.set_planner_id("RRTConnectkConfigDefault")
    group.set_planning_time(10.0)
    group.set_joint_value_target([target[name] for name in active_joints])
    planned = group.plan()
    success = bool(planned[0]) if isinstance(planned, tuple) else True
    trajectory = planned[1] if isinstance(planned, tuple) else planned
    if (not success or not trajectory_is_bounded(
            trajectory, start, maximum_excursion=0.30)):
        raise RuntimeCheckError("MoveIt did not produce a bounded arm plan")
    if not group.execute(trajectory, wait=True):
        raise RuntimeCheckError("MoveIt arm trajectory execution failed")
    group.stop()

    after_message = air_ground.wait_for_condition(
        rospy, "/ground/joint_states", joint_state_type,
        lambda item: positions_within(
            joint_state_summary(
                item, rospy.Time.now().to_sec())["positions"],
            target, 0.08),
        timeout, "MoveIt final arm position")
    after = joint_state_summary(
        after_message, rospy.Time.now().to_sec())["positions"]
    elbow_delta = after["elbow_joint"] - start["elbow_joint"]
    if abs(elbow_delta) < 0.07:
        raise RuntimeCheckError("MoveIt arm did not physically move")
    return {
        "planner": "RRTConnectkConfigDefault",
        "elbow_delta_rad": elbow_delta,
        "trajectory_points": len(trajectory.joint_trajectory.points),
    }


def run_checks(timeout):
    try:
        import actionlib
        import moveit_commander
        import rospy
        import tf2_ros
        from actionlib_msgs.msg import GoalStatus
        from control_msgs.msg import (
            FollowJointTrajectoryAction, FollowJointTrajectoryGoal)
        from controller_manager_msgs.srv import ListControllers
        from gazebo_msgs.msg import ModelStates
        from geometry_msgs.msg import Twist
        from moveit_msgs.msg import RobotState
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import (
            CameraInfo, Image, JointState, LaserScan, PointCloud2)
        from std_msgs.msg import Bool
        from trajectory_msgs.msg import JointTrajectoryPoint
    except ImportError as error:
        raise RuntimeCheckError(
            "ROS Python environment is incomplete: %s" % error)

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node(
        "ground_manipulator_runtime_check", anonymous=True,
        disable_signals=True)
    clock_deadline = time.monotonic() + timeout
    while rospy.Time.now().to_sec() <= 0.0:
        if rospy.is_shutdown() or time.monotonic() >= clock_deadline:
            raise RuntimeCheckError("simulation clock did not start")
        time.sleep(0.05)

    checks = {
        "model": check_model(rospy, ModelStates, timeout),
        "runtime_ready": check_runtime_ready(rospy, Bool, timeout),
        "controllers": check_controllers(rospy, ListControllers, timeout),
    }
    joint_message = current_joint_state(rospy, JointState, timeout)
    checks["joints"] = joint_state_summary(
        joint_message, rospy.Time.now().to_sec())
    checks["d435"] = check_sensors(
        rospy, Image, CameraInfo, PointCloud2, timeout)
    checks["lidar"] = check_ground_scan(rospy, LaserScan, timeout)
    checks["tf"] = air_ground.check_current_tf_frames(
        rospy, tf2_ros, GROUND_TF_FRAMES, timeout)
    checks["controller_motion"] = check_controller_motion(
        rospy, actionlib, JointState, FollowJointTrajectoryAction,
        FollowJointTrajectoryGoal, JointTrajectoryPoint, GoalStatus, timeout)
    checks["base_motion"] = bunker.check_motion(
        rospy, Odometry, Twist, timeout)
    checks["moveit"] = check_moveit(
        rospy, moveit_commander, RobotState, JointState, timeout)
    return checks


def write_summary(path, payload):
    if path is None:
        return
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Check the BUNKER+AUBO+AG95+D435 Gazebo runtime")
    parser.add_argument("--summary", help="write a compact JSON summary")
    parser.add_argument("--timeout", type=float, default=30.0)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if not math.isfinite(args.timeout) or args.timeout <= 0.0:
        print(
            "check-ground-manipulator: --timeout must be positive",
            file=sys.stderr)
        return 64
    payload = {"status": "FAIL"}
    try:
        payload["checks"] = run_checks(args.timeout)
        payload["status"] = "PASS"
    except (RuntimeCheckError, KeyboardInterrupt) as error:
        payload["error"] = str(error)
    write_summary(args.summary, payload)
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if payload["status"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
