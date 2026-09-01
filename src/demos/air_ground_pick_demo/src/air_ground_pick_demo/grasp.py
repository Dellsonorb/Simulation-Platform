"""Minimal geometry and physical-contact helpers for an AG95 top grasp."""

import math
from collections import namedtuple

import numpy as np


class GraspError(RuntimeError):
    pass


Feasibility = namedtuple(
    "Feasibility", "feasible grasp_span required_opening")
CartesianPose = namedtuple("CartesianPose", "position orientation")
GraspPoses = namedtuple("GraspPoses", "pregrasp grasp lift")


def _finite_tuple(values, length, label):
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError, OverflowError) as error:
        raise GraspError("%s must contain finite values" % label) from error
    if len(result) != length or not all(math.isfinite(value)
                                        for value in result):
        raise GraspError("%s must contain finite values" % label)
    return result


def check_target_feasibility(target_size, maximum_opening, opening_margin):
    """Check the side-up target span against the measured AG95 opening."""
    length, grasp_span, height = _finite_tuple(
        target_size, 3, "target_size")
    maximum = float(maximum_opening)
    margin = float(opening_margin)
    if (length <= grasp_span or grasp_span <= 0.0 or height <= 0.0 or
            not math.isfinite(maximum) or maximum <= 0.0 or
            not math.isfinite(margin) or margin < 0.0):
        raise GraspError("target or gripper geometry is invalid")
    required = grasp_span + margin
    return Feasibility(required <= maximum, grasp_span, required)


def conservative_jaw_opening(
        master_joint_position, maximum_opening, maximum_joint_position):
    """Return the conservative chord bound used for the real AG95 stroke."""
    joint = float(master_joint_position)
    opening = float(maximum_opening)
    joint_limit = float(maximum_joint_position)
    if (not math.isfinite(joint) or joint < -1e-4 or
            not math.isfinite(opening) or opening <= 0.0 or
            not math.isfinite(joint_limit) or joint_limit <= 0.0 or
            joint > joint_limit + 1e-4):
        raise GraspError("AG95 joint measurement or calibration is invalid")
    bounded_joint = min(joint_limit, max(0.0, joint))
    return opening * (1.0 - bounded_joint / joint_limit)


def _top_down_quaternion(yaw):
    """Quaternion for tool +X down and tool +Y along the target short axis."""
    half_yaw = 0.5 * yaw
    half_pitch = 0.25 * math.pi
    cp = math.cos(half_pitch)
    sp = math.sin(half_pitch)
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)
    quaternion = (-sp * sy, sp * cy, cp * sy, cp * cy)
    norm = math.sqrt(sum(value * value for value in quaternion))
    return tuple(value / norm for value in quaternion)


def quaternion_matrix(quaternion):
    """Return a 3x3 rotation matrix for an ``(x, y, z, w)`` quaternion."""
    x, y, z, w = _finite_tuple(quaternion, 4, "quaternion")
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        raise GraspError("quaternion has zero norm")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray((
        (1.0 - 2.0 * (y * y + z * z),
         2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w),
         1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w),
         2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)),
    ), dtype=np.float64)


def generate_top_down_grasp(
        target, target_size, pregrasp_height, lift_height,
        finger_pad_lower_edge_offset, contact_overlap, surface_clearance):
    """Generate one vertical pregrasp, physical insertion, and lift sequence."""
    x, y, center_z, yaw = _finite_tuple(target, 4, "target")
    length, span, vertical_height = _finite_tuple(
        target_size, 3, "target_size")
    if length <= span or span <= 0.0 or vertical_height <= 0.0:
        raise GraspError("target geometry is invalid")
    pregrasp_offset = float(pregrasp_height)
    lift_offset = float(lift_height)
    pad_edge = float(finger_pad_lower_edge_offset)
    overlap = float(contact_overlap)
    clearance = float(surface_clearance)
    values = (pregrasp_offset, lift_offset, pad_edge, overlap, clearance)
    if (not all(math.isfinite(value) for value in values) or
            pregrasp_offset <= 0.0 or lift_offset <= 0.0 or
            pad_edge <= 0.0 or overlap <= 0.0 or clearance < 0.0):
        raise GraspError("grasp offsets are invalid")

    top_surface = center_z + 0.5 * vertical_height
    support_plane = center_z - 0.5 * vertical_height
    grasp_z = top_surface - pad_edge - overlap
    if grasp_z < support_plane + clearance:
        raise GraspError("AG95 TCP insertion violates support-plane clearance")
    orientation = _top_down_quaternion(yaw)
    grasp = CartesianPose((x, y, grasp_z), orientation)
    pregrasp = CartesianPose(
        (x, y, grasp_z + pregrasp_offset), orientation)
    lift = CartesianPose((x, y, grasp_z + lift_offset), orientation)
    return GraspPoses(pregrasp, grasp, lift)


def contact_sides(
        contact_pairs, target_marker="pick_target",
        left_pad_marker="left_finger_pad",
        right_pad_marker="right_finger_pad"):
    """Classify target collision pairs by the physical AG95 pad involved."""
    left = False
    right = False
    for pair in contact_pairs:
        try:
            first, second = pair
        except (TypeError, ValueError) as error:
            raise GraspError("contact pair must contain two collision names") \
                from error
        first = str(first)
        second = str(second)
        if target_marker in first:
            other = second
        elif target_marker in second:
            other = first
        else:
            continue
        left = left or left_pad_marker in other
        right = right or right_pad_marker in other
    return left, right


def zero_terminal_motion(trajectory):
    """Make a retimed point-to-point trajectory finish at rest."""
    try:
        joint_trajectory = trajectory.joint_trajectory
        joint_count = len(joint_trajectory.joint_names)
        terminal = joint_trajectory.points[-1]
    except (AttributeError, IndexError, TypeError) as error:
        raise GraspError("trajectory has no terminal joint point") from error
    if joint_count < 1:
        raise GraspError("trajectory has no controlled joints")
    terminal.velocities = [0.0] * joint_count
    terminal.accelerations = [0.0] * joint_count
    return trajectory
