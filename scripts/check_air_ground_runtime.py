#!/usr/bin/env python3
"""Check P450 and the complete Ground Robot in one running Gazebo world."""

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
import check_bunker_runtime as bunker


RuntimeCheckError = bunker.RuntimeCheckError
GROUND_MODEL_NAME = "ground_robot"
GROUND_SCAN_FORWARD_RANGE_M = (1.0, 1.6)

AIR_TF_FRAMES = (
    "uav1/base_link",
    "uav1/camera_link",
    "uav1/camera_depth_frame",
    "uav1/camera_ired1_frame",
    "uav1/camera_ired2_frame",
    "uav1/camera_imu_link",
    "uav1/d435i_link",
    "uav1/camera_color_optical_frame",
    "uav1/camera_depth_optical_frame",
)
GROUND_TF_FRAMES = (
    "ground/base_link",
    "ground/lidar_2d_link",
    "ground/imu_link",
    "ground/aubo_i5_base_link",
    "ground/ee_link",
    "ground/d435_color_optical_frame",
    "ground/d435_depth_optical_frame",
    "ground/gripper_tcp_link",
    "ground/left_finger_pad",
    "ground/right_finger_pad",
)


def sensor_header_summary(message, expected_frame, now):
    stamp = message.header.stamp.to_sec()
    age = now - stamp
    frame = message.header.frame_id.lstrip("/")
    if not math.isfinite(stamp) or stamp <= 0.0:
        raise RuntimeCheckError("sensor timestamp is invalid")
    if frame != expected_frame:
        raise RuntimeCheckError(
            "sensor frame is %s, expected %s" %
            (message.header.frame_id, expected_frame))
    if age < -0.1 or age > 1.0:
        raise RuntimeCheckError(
            "sensor data is stale: stamp=%.6f, now=%.6f, age=%.3fs" %
            (stamp, now, age))
    return {"frame": frame, "stamp": stamp, "age_s": age}


def camera_info_summary(message, expected_frame, now):
    summary = sensor_header_summary(message, expected_frame, now)
    calibration = tuple(message.K) + tuple(message.P)
    if (message.width <= 0 or message.height <= 0 or
            len(message.K) != 9 or len(message.P) != 12 or
            not all(math.isfinite(float(value)) for value in calibration) or
            message.K[0] <= 0.0 or message.K[4] <= 0.0 or
            message.P[0] <= 0.0 or message.P[5] <= 0.0):
        raise RuntimeCheckError("camera calibration is invalid")
    summary.update({"width": message.width, "height": message.height})
    return summary


def transform_summary(transform, expected_target, expected_source, now):
    target = transform.header.frame_id.lstrip("/")
    source = transform.child_frame_id.lstrip("/")
    stamp = transform.header.stamp.to_sec()
    age = now - stamp
    value = transform.transform
    components = (
        value.translation.x, value.translation.y, value.translation.z,
        value.rotation.x, value.rotation.y, value.rotation.z, value.rotation.w,
    )
    quaternion_norm = math.sqrt(sum(
        float(component) ** 2 for component in components[3:]))
    if target != expected_target or source != expected_source:
        raise RuntimeCheckError(
            "TF is %s <- %s, expected %s <- %s" %
            (transform.header.frame_id, transform.child_frame_id,
             expected_target, expected_source))
    if (not math.isfinite(stamp) or stamp <= 0.0 or
            not all(math.isfinite(float(component)) for component in components)):
        raise RuntimeCheckError("TF contains a zero or non-finite value")
    if abs(quaternion_norm - 1.0) > 1e-3:
        raise RuntimeCheckError("TF quaternion is not normalized")
    if age < -0.2 or age > 2.0:
        raise RuntimeCheckError(
            "TF is stale: stamp=%.6f, now=%.6f, age=%.3fs" %
            (stamp, now, age))
    return {
        "chain": "%s<-%s" % (target, source),
        "stamp": stamp,
        "age_s": age,
    }


def wait_for_condition(rospy, topic, message_type, predicate, timeout, label):
    deadline = time.monotonic() + timeout
    last_error = None
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        try:
            message = rospy.wait_for_message(
                topic, message_type, timeout=remaining)
            if predicate(message):
                return message
        except Exception as error:
            last_error = error
    detail = "" if last_error is None else ": %s" % last_error
    raise RuntimeCheckError("timed out waiting for %s%s" % (label, detail))


def wait_for_current_sensor(rospy, topic, message_type, frame, timeout):
    messages = deque(maxlen=2)
    messages_lock = threading.Lock()

    def receive(message):
        with messages_lock:
            messages.append(message)

    subscription = rospy.Subscriber(
        topic, message_type, receive, queue_size=1,
        buff_size=4 * 1024 * 1024, tcp_nodelay=True)
    deadline = time.monotonic() + timeout
    last_error = None
    try:
        while time.monotonic() < deadline:
            with messages_lock:
                current_messages = tuple(messages)
            if len(current_messages) == 2:
                first, second = current_messages
                try:
                    summary = sensor_header_summary(
                        second, frame, rospy.Time.now().to_sec())
                    if bunker.stamp_advanced(
                            first.header.stamp.to_sec(),
                            second.header.stamp.to_sec()):
                        return second, summary
                except RuntimeCheckError as error:
                    last_error = error
            if rospy.is_shutdown():
                break
            time.sleep(0.05)
    finally:
        subscription.unregister()
    detail = "" if last_error is None else ": %s" % last_error
    raise RuntimeCheckError(
        "%s did not produce current advancing data%s" % (topic, detail))


def check_models(rospy, model_states_type, timeout):
    required = {"p450_D435i_0", GROUND_MODEL_NAME}
    message = wait_for_condition(
        rospy, "/gazebo/model_states", model_states_type,
        lambda item: required.issubset(set(item.name)), timeout,
        "P450 and Ground Robot Gazebo models")
    return sorted(required.intersection(message.name))


def valid_p450_state(message):
    return (
        bool(message.connected) and
        bool(message.odom_valid) and
        len(message.position) == 3 and
        all(math.isfinite(float(value)) for value in message.position)
    )


def check_p450_state(rospy, mavros_state_type, uav_state_type, timeout):
    mavros = wait_for_condition(
        rospy, "/uav1/mavros/state", mavros_state_type,
        lambda message: message.connected, timeout, "MAVROS connection")
    state = wait_for_condition(
        rospy, "/uav1/prometheus/state", uav_state_type,
        valid_p450_state,
        timeout, "P450 odometry")
    return {
        "mavros_connected": bool(mavros.connected),
        "armed": bool(mavros.armed),
        "mode": mavros.mode,
        "odom_valid": bool(state.odom_valid),
        "position_m": [float(value) for value in state.position],
    }


def p450_sensor_contracts(image_type, camera_info_type, imu_type):
    return (
        ("color", "/uav1/camera/color/image_raw", image_type,
         "uav1/camera_link", "image"),
        ("color_info", "/uav1/camera/color/camera_info", camera_info_type,
         "uav1/camera_link", "camera_info"),
        ("depth", "/uav1/camera/depth/image_raw", image_type,
         "uav1/camera_depth_frame", "image"),
        ("depth_info", "/uav1/camera/depth/camera_info", camera_info_type,
         "uav1/camera_depth_frame", "camera_info"),
        ("imu", "/uav1/camera/imu", imu_type,
         "uav1/camera_imu_link", "imu"),
    )


def check_p450_sensors(
        rospy, image_type, camera_info_type, imu_type, timeout):
    results = {}
    for label, topic, message_type, frame, kind in p450_sensor_contracts(
            image_type, camera_info_type, imu_type):
        message, summary = wait_for_current_sensor(
            rospy, topic, message_type, frame, timeout)
        if kind == "image":
            if message.width <= 0 or message.height <= 0 or not message.data:
                raise RuntimeCheckError("%s image is empty" % label)
            summary.update({"width": message.width, "height": message.height})
        elif kind == "camera_info":
            summary = camera_info_summary(
                message, frame, rospy.Time.now().to_sec())
        results[label] = summary
    for image_label, info_label in (
            ("color", "color_info"), ("depth", "depth_info")):
        if (results[image_label]["width"], results[image_label]["height"]) != (
                results[info_label]["width"], results[info_label]["height"]):
            raise RuntimeCheckError(
                "%s image and camera info dimensions differ" % image_label)
    return results


def check_current_tf_frames(rospy, tf2_ros, frames, timeout):
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    observed = []
    for source in frames:
        deadline = time.monotonic() + timeout
        last_error = None
        while time.monotonic() < deadline:
            remaining = max(0.01, deadline - time.monotonic())
            try:
                transform = buffer.lookup_transform(
                    "map", source, rospy.Time(0),
                    rospy.Duration(min(0.5, remaining)))
                observed.append(transform_summary(
                    transform, "map", source, rospy.Time.now().to_sec()))
                break
            except Exception as error:
                last_error = error
                time.sleep(0.05)
        else:
            raise RuntimeCheckError(
                "missing current TF map <- %s: %s" %
                (source, last_error))
    # Keep the listener alive until every lookup has completed.
    _ = listener
    return observed


def check_air_tf(rospy, tf2_ros, timeout):
    return check_current_tf_frames(
        rospy, tf2_ros, AIR_TF_FRAMES, timeout)


def check_ground_tf(rospy, tf2_ros, timeout):
    return check_current_tf_frames(
        rospy, tf2_ros, GROUND_TF_FRAMES, timeout)


def check_ground_joints(
        rospy, ground, joint_state_type, timeout):
    message = ground.current_joint_state(
        rospy, joint_state_type, timeout)
    return ground.joint_state_summary(
        message, rospy.Time.now().to_sec())


def run_checks(timeout):
    try:
        import check_ground_manipulator_runtime as ground
        import rospy
        import tf2_ros
        from bunker_msgs.msg import BunkerStatus
        from controller_manager_msgs.srv import ListControllers
        from gazebo_msgs.msg import ModelStates
        from geometry_msgs.msg import Twist
        from mavros_msgs.msg import State
        from nav_msgs.msg import Odometry
        from prometheus_msgs.msg import UAVState
        from sensor_msgs.msg import (
            CameraInfo, Image, Imu, JointState, LaserScan, PointCloud2)
        from std_msgs.msg import Bool
    except ImportError as error:
        raise RuntimeCheckError("ROS Python environment is incomplete: %s" % error)

    rospy.init_node(
        "air_ground_runtime_check", anonymous=True, disable_signals=True)
    deadline = time.monotonic() + timeout
    while rospy.Time.now().to_sec() <= 0.0:
        if rospy.is_shutdown() or time.monotonic() >= deadline:
            raise RuntimeCheckError("simulation clock did not start")
        time.sleep(0.05)

    checks = {"models": check_models(rospy, ModelStates, timeout)}
    checks["p450"] = {
        "state": check_p450_state(rospy, State, UAVState, timeout),
        "sensors": check_p450_sensors(
            rospy, Image, CameraInfo, Imu, timeout),
        "tf": check_air_tf(rospy, tf2_ros, timeout),
    }
    checks["ground"] = {
        "runtime_ready": ground.check_runtime_ready(
            rospy, Bool, timeout),
        "controllers": ground.check_controllers(
            rospy, ListControllers, timeout),
        "joints": check_ground_joints(
            rospy, ground, JointState, timeout),
        "d435": ground.check_sensors(
            rospy, Image, CameraInfo, PointCloud2, timeout),
        "scan": ground.check_ground_scan(
            rospy, LaserScan, timeout,
            forward_range_bounds=GROUND_SCAN_FORWARD_RANGE_M),
        "imu": bunker.check_imu(rospy, Imu, timeout),
        "bunker_status": bunker.check_status(
            rospy, BunkerStatus, timeout),
        "tf": check_ground_tf(rospy, tf2_ros, timeout),
        "motion": bunker.check_motion(
            rospy, Odometry, Twist, timeout),
    }
    return checks


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Check a joint P450 and Ground Robot Gazebo runtime")
    parser.add_argument("--summary", help="write a compact JSON summary")
    parser.add_argument("--timeout", type=float, default=30.0)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if not math.isfinite(args.timeout) or args.timeout <= 0.0:
        print("check-air-ground: --timeout must be positive", file=sys.stderr)
        return 64
    payload = {"status": "FAIL"}
    try:
        payload["checks"] = run_checks(args.timeout)
        payload["status"] = "PASS"
    except (RuntimeCheckError, KeyboardInterrupt) as error:
        payload["error"] = str(error)
    if args.summary:
        output = Path(args.summary)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n",
            encoding="utf-8")
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if payload["status"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
