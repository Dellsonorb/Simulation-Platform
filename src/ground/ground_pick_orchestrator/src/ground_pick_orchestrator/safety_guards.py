"""ROS-independent safety checks used at mission component boundaries."""
from __future__ import division

import math


def _finite(value):
    return not math.isnan(float(value)) and not math.isinf(float(value))


def motion_is_stopped(odom_motion, command_motion,
                      linear_threshold, angular_threshold):
    if odom_motion is None or command_motion is None:
        return False
    values = tuple(odom_motion) + tuple(command_motion)
    if not all(_finite(value) for value in values):
        return False
    odom_x, odom_y, odom_yaw = odom_motion
    command_x, command_yaw = command_motion
    return (
        abs(odom_x) <= float(linear_threshold) and
        abs(odom_y) <= float(linear_threshold) and
        abs(command_x) <= float(linear_threshold) and
        abs(odom_yaw) <= float(angular_threshold) and
        abs(command_yaw) <= float(angular_threshold))


def refined_pose_is_valid(values):
    if len(values) != 8 or not all(_finite(value) for value in values):
        return False
    quaternion_norm = math.sqrt(sum(float(value) ** 2
                                    for value in values[4:8]))
    return 0.99 <= quaternion_norm <= 1.01


def joint_window_is_settled(samples, targets, goal_tolerance,
                            position_delta_tolerance, minimum_span):
    """Confirm arrival from a time window of actual joint positions.

    Gazebo's differentiated JointState velocity can contain isolated spikes
    even when the measured position is stationary.  A bounded position range
    over a minimum interval directly checks the property needed before camera
    sampling, while every sample must remain near the commanded target.
    """
    if len(samples) < 2 or not targets:
        return False
    values = (goal_tolerance, position_delta_tolerance, minimum_span)
    if not all(_finite(value) and float(value) >= 0.0 for value in values):
        return False
    if float(samples[-1][0]) - float(samples[0][0]) < float(minimum_span):
        return False
    for unused_stamp, positions in samples:
        if any(name not in positions or not _finite(positions[name])
               for name in targets):
            return False
        if max(abs(float(positions[name]) - float(target))
               for name, target in targets.items()) > float(goal_tolerance):
            return False
    for name in targets:
        measured = [float(positions[name]) for unused_stamp, positions in samples]
        if max(measured) - min(measured) > float(position_delta_tolerance):
            return False
    return True


def observation_result_is_safe(exit_code, gate_verified):
    """Accept mover's racy one-frame check only after gate verification."""
    return int(exit_code) == 0 or (int(exit_code) == 6 and gate_verified)


def _wrapped_angle_delta(first, second):
    return math.atan2(math.sin(float(second) - float(first)),
                      math.cos(float(second) - float(first)))


def terminal_stop_is_safe(command_motion, command_fresh,
                          first_model_state, second_model_state, sample_span,
                          linear_threshold, angular_threshold):
    """Require a fresh zero command and physically stationary Gazebo model.

    Model samples are ``(x, y, yaw, vx, vy, wz)``.  Checking both reported
    velocity and displacement protects the harness from stale/default cmd_vel
    messages and from a model that keeps drifting after cancellation.
    """
    if command_motion is None or not command_fresh:
        return False
    if first_model_state is None or second_model_state is None:
        return False
    if float(sample_span) < 0.3:
        return False
    values = (tuple(command_motion) + tuple(first_model_state) +
              tuple(second_model_state) + (sample_span,))
    if not all(_finite(value) for value in values):
        return False
    linear = float(linear_threshold)
    angular = float(angular_threshold)
    command_x, command_yaw = command_motion
    first_x, first_y, first_yaw, first_vx, first_vy, first_wz = \
        first_model_state
    second_x, second_y, second_yaw, second_vx, second_vy, second_wz = \
        second_model_state
    return (
        abs(command_x) <= linear and abs(command_yaw) <= angular and
        abs(first_vx) <= linear and abs(first_vy) <= linear and
        abs(second_vx) <= linear and abs(second_vy) <= linear and
        abs(first_wz) <= angular and abs(second_wz) <= angular and
        math.hypot(second_x - first_x, second_y - first_y) <= linear and
        abs(_wrapped_angle_delta(first_yaw, second_yaw)) <= angular)
