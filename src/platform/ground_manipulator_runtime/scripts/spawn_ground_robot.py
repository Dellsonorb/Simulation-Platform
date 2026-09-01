#!/usr/bin/env python3

import math
import sys
import threading
import time

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from gazebo_msgs.msg import ModelStates
from gazebo_ros import gazebo_interface
from geometry_msgs.msg import Pose, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler
from trajectory_msgs.msg import JointTrajectoryPoint

from ground_manipulator_runtime.startup import (
    StartupError,
    arm_home_trajectory,
    initialize_ground_robot,
    joint_feedback_is_complete,
)


class GazeboSpawnAttempt:
    def __init__(self, spawn_call):
        self._accepted = None
        self._error = None
        self._done = threading.Event()
        self._thread = threading.Thread(target=self._run, args=(spawn_call,))
        self._thread.daemon = True
        self._thread.start()

    def _run(self, spawn_call):
        try:
            self._accepted = spawn_call()
        except Exception as error:
            self._error = error
        finally:
            self._done.set()

    def result(self, timeout):
        if not self._done.wait(timeout):
            raise StartupError("Gazebo spawn service did not return")
        if self._error is not None:
            raise StartupError(str(self._error))
        return self._accepted


class GazeboGateway:
    def begin_spawn(self, model_name, robot_xml, initial_pose):
        namespace = rospy.get_namespace().rstrip("/") or "/"
        return GazeboSpawnAttempt(
            lambda: gazebo_interface.spawn_urdf_model_client(
                model_name, robot_xml, namespace, initial_pose, "world",
                "/gazebo"))

def wait_for_model(model_name, timeout):
    deadline = time.monotonic() + timeout
    while not rospy.is_shutdown():
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return False
        try:
            message = rospy.wait_for_message(
                "/gazebo/model_states", ModelStates,
                timeout=min(1.0, remaining))
        except rospy.ROSException:
            continue
        if model_name in message.name:
            return True
    return False


def trajectory_goal(stages):
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = [
        name for name, _value in stages[0][1]]
    for duration, positions in stages:
        point = JointTrajectoryPoint()
        point.positions = [value for _name, value in positions]
        point.velocities = [0.0] * len(positions)
        point.time_from_start = rospy.Duration(duration)
        goal.trajectory.points.append(point)
    return goal


def feedback_logger(name):
    last_log_time = [0.0]

    def log_feedback(message):
        now = time.monotonic()
        if now - last_log_time[0] < 0.75:
            return
        last_log_time[0] = now
        rospy.loginfo(
            "Ground %s home feedback: desired=%s actual=%s error=%s",
            name, list(message.desired.positions),
            list(message.actual.positions), list(message.error.positions))

    return log_feedback


def wait_for_controller_feedback(positions, deadline):
    while not rospy.is_shutdown():
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return False
        try:
            message = rospy.wait_for_message(
                "joint_states", JointState, timeout=min(1.0, remaining))
        except rospy.ROSException:
            continue
        if joint_feedback_is_complete(message, positions):
            return True
    return False


def drive_home(positions, timeout):
    arm_positions = positions[:-1]
    gripper_positions = positions[-1:]
    arm = actionlib.SimpleActionClient(
        "arm_controller/follow_joint_trajectory",
        FollowJointTrajectoryAction)
    gripper = actionlib.SimpleActionClient(
        "gripper_controller/follow_joint_trajectory",
        FollowJointTrajectoryAction)
    deadline = time.monotonic() + timeout
    named_clients = (("arm", arm), ("gripper", gripper))
    for name, client in named_clients:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0 or not client.wait_for_server(
                rospy.Duration(remaining)):
            rospy.logerr(
                "Ground %s trajectory action server was not ready", name)
            return False
    if not wait_for_controller_feedback(positions, deadline):
        rospy.logerr(
            "Ground controller joint feedback was not ready before home")
        return False
    arm.send_goal(
        trajectory_goal(arm_home_trajectory(arm_positions)),
        feedback_cb=feedback_logger("arm"))
    gripper.send_goal(
        trajectory_goal(((2.0, gripper_positions),)),
        feedback_cb=feedback_logger("gripper"))
    for name, client in named_clients:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0 or not client.wait_for_result(
                rospy.Duration(remaining)):
            client.cancel_goal()
            rospy.logerr(
                "Ground %s home trajectory timed out: state=%s status=%s",
                name, client.get_state(), client.get_goal_status_text())
            return False
        if client.get_state() != GoalStatus.SUCCEEDED:
            rospy.logerr(
                "Ground %s home trajectory failed: state=%s status=%s "
                "result=%s",
                name, client.get_state(), client.get_goal_status_text(),
                client.get_result())
            return False
    return True


def parameter_pose():
    values = {
        name: float(rospy.get_param("~" + name))
        for name in ("x", "y", "z", "roll", "pitch", "yaw")
    }
    if not all(math.isfinite(value) for value in values.values()):
        raise StartupError("ground robot pose must be finite")
    quaternion = quaternion_from_euler(
        values["roll"], values["pitch"], values["yaw"])
    pose = Pose()
    pose.position.x = values["x"]
    pose.position.y = values["y"]
    pose.position.z = values["z"]
    pose.orientation = Quaternion(*quaternion)
    return pose


def main():
    rospy.init_node("spawn_ground_robot")
    model_spawned = rospy.Publisher(
        "model_spawned", Bool, queue_size=1, latch=True)
    runtime_ready = rospy.Publisher(
        "runtime_ready", Bool, queue_size=1, latch=True)
    try:
        model_timeout = float(rospy.get_param("~model_timeout", 30.0))
        controller_timeout = float(
            rospy.get_param("~controller_timeout", 30.0))
        accepted = initialize_ground_robot(
            GazeboGateway(), rospy.get_param("robot_description"),
            parameter_pose(), wait_for_model,
            lambda: model_spawned.publish(Bool(data=True)),
            lambda positions, _timeout: drive_home(
                positions, controller_timeout),
            timeout=model_timeout)
    except (KeyError, TypeError, ValueError, StartupError) as error:
        rospy.logerr("Ground robot startup failed: %s", error)
        return 1
    if not accepted:
        rospy.logwarn(
            "Gazebo reported a spawn timeout, but ground_robot is present; "
            "continuing after verified joint initialization")
    runtime_ready.publish(Bool(data=True))
    rospy.loginfo("Ground robot model and controller-driven home pose are ready")
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
