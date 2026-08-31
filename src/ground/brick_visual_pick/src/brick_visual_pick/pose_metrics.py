"""Pose-error helpers for visual-pick validation."""
from __future__ import division

import math

import tf.transformations as transformations

from brick_visual_pick.quality_gate import yaw_error_mod_pi


def _yaw(pose):
    orientation = pose.orientation
    return transformations.euler_from_quaternion((
        orientation.x, orientation.y, orientation.z, orientation.w))[2]


def pose_error(estimate, reference):
    dx = estimate.position.x - reference.position.x
    dy = estimate.position.y - reference.position.y
    dz = estimate.position.z - reference.position.z
    return {
        'xy': math.sqrt(dx * dx + dy * dy),
        'z': abs(dz),
        'position': math.sqrt(dx * dx + dy * dy + dz * dz),
        'yaw': yaw_error_mod_pi(_yaw(estimate), _yaw(reference)),
    }
