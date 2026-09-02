#!/usr/bin/env python3
"""Exercise a running BUNKER simulation through its public ROS interfaces."""

import argparse
import json
import math
from pathlib import Path
import sys
import time


class RuntimeCheckError(RuntimeError):
    pass


def yaw_from_quaternion(x, y, z, w):
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def wrapped_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def relative_planar_motion(start, end):
    start_x, start_y, start_yaw = start
    end_x, end_y, end_yaw = end
    dx = end_x - start_x
    dy = end_y - start_y
    forward = math.cos(start_yaw) * dx + math.sin(start_yaw) * dy
    lateral = -math.sin(start_yaw) * dx + math.cos(start_yaw) * dy
    return forward, lateral, wrapped_angle(end_yaw - start_yaw)


def stamp_advanced(first, second):
    return math.isfinite(first) and math.isfinite(second) and second > first


def current_header_summary(message, expected_frame, now):
    stamp = message.header.stamp.to_sec()
    frame = message.header.frame_id.lstrip("/")
    age = now - stamp
    if not math.isfinite(stamp) or stamp <= 0.0:
        raise RuntimeCheckError("message timestamp is invalid")
    if frame != expected_frame:
        raise RuntimeCheckError(
            "message frame is %s, expected %s" %
            (message.header.frame_id, expected_frame))
    if not math.isfinite(age) or age < -0.1 or age > 1.0:
        raise RuntimeCheckError(
            "message is stale: stamp=%.6f now=%.6f age=%.3fs" %
            (stamp, now, age))
    return {"frame": frame, "stamp": stamp, "age_s": age}


def imu_summary(message, now):
    summary = current_header_summary(message, "ground/imu_link", now)
    orientation = message.orientation
    angular = message.angular_velocity
    acceleration = message.linear_acceleration
    values = (
        orientation.x, orientation.y, orientation.z, orientation.w,
        angular.x, angular.y, angular.z,
        acceleration.x, acceleration.y, acceleration.z,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise RuntimeCheckError("/ground/imu/data contains non-finite values")
    quaternion_norm = math.sqrt(sum(
        float(value) ** 2 for value in values[:4]))
    if abs(quaternion_norm - 1.0) > 1e-3:
        raise RuntimeCheckError("/ground/imu/data quaternion is invalid")
    summary.update({
        "angular_velocity_radps": [
            float(angular.x), float(angular.y), float(angular.z)],
        "linear_acceleration_mps2": [
            float(acceleration.x), float(acceleration.y),
            float(acceleration.z)],
    })
    return summary


def bunker_status_summary(message, now):
    summary = current_header_summary(message, "ground/base_link", now)
    values = (
        message.linear_velocity, message.angular_velocity,
        message.battery_voltage,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise RuntimeCheckError(
            "/ground/bunker_status contains non-finite values")
    summary.update({
        "linear_velocity_mps": float(message.linear_velocity),
        "angular_velocity_radps": float(message.angular_velocity),
        "base_state": int(message.base_state),
        "control_mode": int(message.control_mode),
        "fault_code": int(message.fault_code),
        "battery_voltage": float(message.battery_voltage),
    })
    return summary


def odometry_summary(message, now):
    summary = current_header_summary(message, "ground/odom", now)
    child = message.child_frame_id.lstrip("/")
    if child != "ground/base_link":
        raise RuntimeCheckError(
            "/ground/odom child frame is %s, expected ground/base_link" %
            message.child_frame_id)
    pose = message.pose.pose
    twist = message.twist.twist
    values = (
        pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w,
        twist.linear.x, twist.linear.y, twist.linear.z,
        twist.angular.x, twist.angular.y, twist.angular.z,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise RuntimeCheckError("/ground/odom contains non-finite values")
    summary["child_frame"] = child
    return summary


def pose_from_odometry(message):
    position = message.pose.pose.position
    orientation = message.pose.pose.orientation
    return (
        position.x,
        position.y,
        yaw_from_quaternion(
            orientation.x, orientation.y, orientation.z, orientation.w),
    )


def wait_for_connection(rospy, publisher, timeout):
    deadline = time.monotonic() + timeout
    while publisher.get_num_connections() == 0:
        if rospy.is_shutdown() or time.monotonic() >= deadline:
            raise RuntimeCheckError("/ground/cmd_vel has no subscriber")
        time.sleep(0.05)


def wait_for_message(rospy, topic, message_type, timeout):
    try:
        return rospy.wait_for_message(topic, message_type, timeout=timeout)
    except Exception as error:
        raise RuntimeCheckError("no fresh message on %s: %s" % (topic, error))


def publish_for_sim_duration(rospy, publisher, message, duration, timeout):
    start = rospy.Time.now().to_sec()
    wall_deadline = time.monotonic() + timeout
    while rospy.Time.now().to_sec() - start < duration:
        if rospy.is_shutdown() or time.monotonic() >= wall_deadline:
            raise RuntimeCheckError("simulation clock stopped during motion")
        publisher.publish(message)
        time.sleep(0.05)


def stop_robot(rospy, publisher, twist_type, timeout):
    zero = twist_type()
    publish_for_sim_duration(rospy, publisher, zero, 0.65, timeout)


def check_scan(rospy, laser_scan_type, timeout):
    first = wait_for_message(rospy, "/ground/scan", laser_scan_type, timeout)
    deadline = time.monotonic() + timeout
    second = first
    first_stamp = first.header.stamp.to_sec()
    age = math.inf
    while (not stamp_advanced(first_stamp, second.header.stamp.to_sec()) or
           age < -0.1 or age > 1.0):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise RuntimeCheckError(
                "/ground/scan did not produce a current advancing frame")
        second = wait_for_message(
            rospy, "/ground/scan", laser_scan_type, remaining)
        age = rospy.Time.now().to_sec() - second.header.stamp.to_sec()
    finite = [value for value in second.ranges if math.isfinite(value)]
    if len(second.ranges) != 720 or not finite:
        raise RuntimeCheckError("/ground/scan has invalid samples")
    if not all(second.range_min <= value <= second.range_max for value in finite):
        raise RuntimeCheckError("/ground/scan contains out-of-range values")
    if second.header.frame_id != "ground/lidar_2d_link":
        raise RuntimeCheckError(
            "/ground/scan has wrong frame: %s" % second.header.frame_id)
    nearest = min(finite)
    if not 1.5 <= nearest <= 2.5:
        raise RuntimeCheckError(
            "/ground/scan does not report the world obstacle: %.3f m" %
            nearest)
    return {
        "frame": second.header.frame_id,
        "samples": len(second.ranges),
        "finite_samples": len(finite),
        "nearest_range_m": nearest,
        "age_s": age,
    }


def check_advancing_topic(
        rospy, topic, message_type, validator, timeout):
    first = wait_for_message(rospy, topic, message_type, timeout)
    first_stamp = first.header.stamp.to_sec()
    deadline = time.monotonic() + timeout
    last_error = None
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        second = wait_for_message(rospy, topic, message_type, remaining)
        if not stamp_advanced(
                first_stamp, second.header.stamp.to_sec()):
            continue
        try:
            return validator(second, rospy.Time.now().to_sec())
        except RuntimeCheckError as error:
            last_error = error
    detail = "" if last_error is None else ": %s" % last_error
    raise RuntimeCheckError(
        "%s did not produce current advancing data%s" % (topic, detail))


def check_imu(rospy, imu_type, timeout):
    return check_advancing_topic(
        rospy, "/ground/imu/data", imu_type, imu_summary, timeout)


def check_status(rospy, status_type, timeout):
    return check_advancing_topic(
        rospy, "/ground/bunker_status", status_type,
        bunker_status_summary, timeout)


def check_odom(rospy, odometry_type, timeout):
    return check_advancing_topic(
        rospy, "/ground/odom", odometry_type,
        odometry_summary, timeout)


def check_tf(rospy, tf2_ros, timeout):
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    pairs = (
        ("map", "ground/base_link"),
        ("ground/base_link", "ground/lidar_2d_link"),
        ("ground/base_link", "ground/imu_link"),
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
    # Keep the listener alive until both lookups have completed.
    _ = listener
    return observed


def check_model(rospy, model_states_type, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        states = wait_for_message(
            rospy, "/gazebo/model_states", model_states_type,
            remaining)
        if "bunker" in states.name:
            return True
    raise RuntimeCheckError("Gazebo does not contain model 'bunker'")


def check_motion(rospy, odometry_type, twist_type, timeout):
    publisher = rospy.Publisher("/ground/cmd_vel", twist_type, queue_size=1)
    wait_for_connection(rospy, publisher, timeout)

    start_message = wait_for_message(rospy, "/ground/odom", odometry_type, timeout)
    start = pose_from_odometry(start_message)
    command = twist_type()
    command.linear.x = 0.30
    publish_for_sim_duration(rospy, publisher, command, 2.1, timeout)
    stop_robot(rospy, publisher, twist_type, timeout)
    forward_end = pose_from_odometry(
        wait_for_message(rospy, "/ground/odom", odometry_type, timeout))
    forward, lateral, yaw_drift = relative_planar_motion(start, forward_end)
    if forward < 0.45 or abs(lateral) > 0.08 or abs(yaw_drift) > 0.15:
        raise RuntimeCheckError(
            "forward motion invalid: %.3f m, lateral %.3f m, yaw %.3f rad" %
            (forward, lateral, yaw_drift))

    turn_start = forward_end
    command = twist_type()
    command.angular.z = 0.60
    publish_for_sim_duration(rospy, publisher, command, 1.7, timeout)
    stop_robot(rospy, publisher, twist_type, timeout)
    turn_end = pose_from_odometry(
        wait_for_message(rospy, "/ground/odom", odometry_type, timeout))
    turn_forward, turn_lateral, turn = relative_planar_motion(
        turn_start, turn_end)
    turn_drift = math.hypot(turn_forward, turn_lateral)
    if turn < 0.75 or turn_drift > 0.10:
        raise RuntimeCheckError(
            "rotation invalid: %.3f rad with %.3f m drift" %
            (turn, turn_drift))

    settled_start = turn_end
    publish_for_sim_duration(rospy, publisher, twist_type(), 0.45, timeout)
    settled_end = pose_from_odometry(
        wait_for_message(rospy, "/ground/odom", odometry_type, timeout))
    coast_forward, coast_lateral, coast_turn = relative_planar_motion(
        settled_start, settled_end)
    if math.hypot(coast_forward, coast_lateral) > 0.03 or abs(coast_turn) > 0.04:
        raise RuntimeCheckError("BUNKER did not stop after zero command")

    return {
        "forward_m": forward,
        "forward_lateral_m": lateral,
        "forward_yaw_rad": yaw_drift,
        "turn_rad": turn,
        "turn_drift_m": turn_drift,
    }


def run_checks(timeout):
    try:
        import rospy
        import tf2_ros
        from bunker_msgs.msg import BunkerStatus
        from gazebo_msgs.msg import ModelStates
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu, LaserScan
    except ImportError as error:
        raise RuntimeCheckError("ROS Python environment is incomplete: %s" % error)

    rospy.init_node("bunker_runtime_check", anonymous=True, disable_signals=True)
    clock_deadline = time.monotonic() + timeout
    while rospy.Time.now().to_sec() <= 0.0:
        if rospy.is_shutdown() or time.monotonic() >= clock_deadline:
            raise RuntimeCheckError("simulation clock did not start")
        time.sleep(0.05)

    return {
        "model": check_model(rospy, ModelStates, timeout),
        "odom": check_odom(rospy, Odometry, timeout),
        "scan": check_scan(rospy, LaserScan, timeout),
        "imu": check_imu(rospy, Imu, timeout),
        "status": check_status(rospy, BunkerStatus, timeout),
        "tf": check_tf(rospy, tf2_ros, timeout),
        "motion": check_motion(rospy, Odometry, Twist, timeout),
    }


def write_summary(path, payload):
    if path is None:
        return
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n",
                      encoding="utf-8")


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Check a running BUNKER Gazebo runtime")
    parser.add_argument("--summary", help="write the small JSON summary here")
    parser.add_argument(
        "--timeout", type=float, default=20.0,
        help="maximum wall seconds for each startup or motion wait")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if not math.isfinite(args.timeout) or args.timeout <= 0.0:
        print("check-bunker-runtime: --timeout must be positive", file=sys.stderr)
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
