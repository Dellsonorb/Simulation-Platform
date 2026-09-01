#!/usr/bin/env python3
"""Check the P450 and BUNKER interfaces in one running Gazebo world."""

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
    required = {"p450_D435i_0", "bunker"}
    message = wait_for_condition(
        rospy, "/gazebo/model_states", model_states_type,
        lambda item: required.issubset(set(item.name)), timeout,
        "P450 and BUNKER Gazebo models")
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


def check_p450_sensors(rospy, image_type, imu_type, timeout):
    topics = (
        ("color", "/uav1/camera/color/image_raw", image_type,
         "uav1/camera_link"),
        ("depth", "/uav1/camera/depth/image_raw", image_type,
         "uav1/camera_depth_frame"),
        ("imu", "/uav1/camera/imu", imu_type, "uav1/camera_imu_link"),
    )
    results = {}
    for label, topic, message_type, frame in topics:
        message, summary = wait_for_current_sensor(
            rospy, topic, message_type, frame, timeout)
        if label != "imu":
            if message.width <= 0 or message.height <= 0 or not message.data:
                raise RuntimeCheckError("%s image is empty" % label)
            summary.update({"width": message.width, "height": message.height})
        results[label] = summary
    return results


def check_air_tf(rospy, tf2_ros, timeout):
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    pairs = (
        ("world", "uav1/base_link"),
        ("uav1/base_link", "uav1/camera_link"),
        ("uav1/camera_link", "uav1/camera_depth_frame"),
    )
    observed = []
    for target, source in pairs:
        try:
            buffer.lookup_transform(
                target, source, rospy.Time(0), rospy.Duration(timeout))
        except Exception as error:
            raise RuntimeCheckError(
                "missing TF %s <- %s: %s" % (target, source, error))
        observed.append("%s<-%s" % (target, source))
    _ = listener
    return observed


def run_checks(timeout):
    try:
        import rospy
        import tf2_ros
        from gazebo_msgs.msg import ModelStates
        from geometry_msgs.msg import Twist
        from mavros_msgs.msg import State
        from nav_msgs.msg import Odometry
        from prometheus_msgs.msg import UAVState
        from sensor_msgs.msg import Image, Imu, LaserScan
    except ImportError as error:
        raise RuntimeCheckError("ROS Python environment is incomplete: %s" % error)

    rospy.init_node(
        "air_ground_runtime_check", anonymous=True, disable_signals=True)
    deadline = time.monotonic() + timeout
    while rospy.Time.now().to_sec() <= 0.0:
        if rospy.is_shutdown() or time.monotonic() >= deadline:
            raise RuntimeCheckError("simulation clock did not start")
        time.sleep(0.05)

    return {
        "models": check_models(rospy, ModelStates, timeout),
        "p450": {
            "state": check_p450_state(rospy, State, UAVState, timeout),
            "sensors": check_p450_sensors(rospy, Image, Imu, timeout),
            "tf": check_air_tf(rospy, tf2_ros, timeout),
        },
        "bunker": {
            "scan": bunker.check_scan(rospy, LaserScan, timeout),
            "tf": bunker.check_tf(rospy, tf2_ros, timeout),
            "motion": bunker.check_motion(
                rospy, Odometry, Twist, timeout),
        },
    }


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Check a joint P450 and BUNKER Gazebo runtime")
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
