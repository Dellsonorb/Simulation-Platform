#!/usr/bin/env python3
"""Spawn the physical pick target after the ground robot is fully ready."""

import math
import os
import time

from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler


class TargetSpawnError(RuntimeError):
    """Raised when the target cannot be inserted within bounded startup time."""


def positive_param(name, default):
    value = float(rospy.get_param(name, default))
    if not math.isfinite(value) or value <= 0.0:
        raise TargetSpawnError("%s must be finite and positive" % name)
    return value


def finite_param(name, default):
    value = float(rospy.get_param(name, default))
    if not math.isfinite(value):
        raise TargetSpawnError("%s must be finite" % name)
    return value


def wait_for_runtime_ready(runtime_ready_topic, timeout):
    deadline = time.monotonic() + timeout
    while not rospy.is_shutdown():
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise TargetSpawnError(
                "timed out waiting for true on %s" % runtime_ready_topic)
        try:
            ready = rospy.wait_for_message(
                runtime_ready_topic, Bool, timeout=min(remaining, 1.0))
        except rospy.ROSException:
            continue
        if ready.data:
            return
        rospy.rostime.wallsleep(min(0.1, remaining))
    raise TargetSpawnError("ROS shut down before ground runtime became ready")


def load_model_xml(model_path):
    resolved = os.path.realpath(os.path.expanduser(model_path))
    if not os.path.isfile(resolved):
        raise TargetSpawnError("target model does not exist: %s" % resolved)
    with open(resolved, "r", encoding="utf-8") as stream:
        model_xml = stream.read()
    if not model_xml.strip():
        raise TargetSpawnError("target model is empty: %s" % resolved)
    return model_xml


def wait_for_sensor(topic, timeout):
    try:
        rospy.wait_for_message(topic, Image, timeout=timeout)
    except rospy.ROSException as error:
        raise TargetSpawnError(
            "timed out waiting for camera frame on %s: %s" %
            (topic, error))


def main():
    rospy.init_node("spawn_pick_target")
    try:
        runtime_ready_topic = rospy.get_param(
            "~runtime_ready_topic", "/ground/runtime_ready")
        runtime_timeout = positive_param("~runtime_timeout", 90.0)
        sensor_timeout = positive_param("~sensor_timeout", 30.0)
        spawn_timeout = positive_param("~spawn_timeout", 30.0)
        air_image_topic = rospy.get_param(
            "~air_image_topic", "/uav1/camera/color/image_raw")
        ground_image_topic = rospy.get_param(
            "~ground_image_topic", "/ground/d435/color/image_raw")
        model_path = rospy.get_param("~model_path")
        model_name = rospy.get_param("~model_name", "pick_target")
        reference_frame = rospy.get_param("~reference_frame", "world")

        initial_pose = Pose()
        initial_pose.position.x = finite_param("~x", 2.0)
        initial_pose.position.y = finite_param("~y", 0.0)
        initial_pose.position.z = finite_param("~z", 0.0575)
        quaternion = quaternion_from_euler(
            finite_param("~roll", 0.0),
            finite_param("~pitch", 0.0),
            finite_param("~yaw", 0.0))
        (initial_pose.orientation.x, initial_pose.orientation.y,
         initial_pose.orientation.z, initial_pose.orientation.w) = quaternion

        wait_for_runtime_ready(runtime_ready_topic, runtime_timeout)
        wait_for_sensor(air_image_topic, sensor_timeout)
        wait_for_sensor(ground_image_topic, sensor_timeout)
        model_xml = load_model_xml(model_path)
        service_name = "/gazebo/spawn_sdf_model"
        rospy.wait_for_service(service_name, timeout=spawn_timeout)
        spawn = rospy.ServiceProxy(service_name, SpawnModel)
        response = spawn(
            model_name=model_name,
            model_xml=model_xml,
            robot_namespace="/pick_target",
            initial_pose=initial_pose,
            reference_frame=reference_frame)
        if not response.success:
            raise TargetSpawnError(
                "Gazebo rejected target: %s" % response.status_message)
        rospy.loginfo(
            "Spawned %s after ground runtime and both D435 streams became ready",
            model_name)
    except (KeyError, OSError, rospy.ROSException,
            rospy.ServiceException, TargetSpawnError) as error:
        rospy.logfatal("Failed to spawn pick target: %s", error)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
