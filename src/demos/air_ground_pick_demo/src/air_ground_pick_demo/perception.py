"""Small, sensor-only RGB-D geometry for the physical pick target."""

import math

import cv2
import numpy as np


class PerceptionError(RuntimeError):
    pass


def _camera_matrix(value):
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.shape == (9,):
        matrix = matrix.reshape(3, 3)
    if (matrix.shape != (3, 3) or not np.isfinite(matrix).all() or
            matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0 or
            abs(matrix[2, 2]) <= 1e-12):
        raise PerceptionError("camera calibration is invalid")
    return matrix


def select_red_component(
        rgb, min_pixels=80, ambiguity_ratio=0.65,
        min_saturation=80, min_value=40):
    """Return the single dominant red component in one RGB image."""
    image = np.asarray(rgb)
    if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
        raise PerceptionError("RGB image must be uint8 HxWx3")
    if (int(min_pixels) <= 0 or not 0.0 <= float(ambiguity_ratio) <= 1.0 or
            not 0 <= int(min_saturation) <= 255 or
            not 0 <= int(min_value) <= 255):
        raise PerceptionError("red component thresholds are invalid")

    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    red = cv2.inRange(
        hsv, np.array((0, min_saturation, min_value), dtype=np.uint8),
        np.array((12, 255, 255), dtype=np.uint8))
    red |= cv2.inRange(
        hsv, np.array((165, min_saturation, min_value), dtype=np.uint8),
        np.array((179, 255, 255), dtype=np.uint8))
    count, labels, statistics, _centroids = cv2.connectedComponentsWithStats(
        red, connectivity=8)
    candidates = sorted(
        (int(statistics[index, cv2.CC_STAT_AREA]), index)
        for index in range(1, count)
        if int(statistics[index, cv2.CC_STAT_AREA]) >= int(min_pixels))
    if not candidates:
        raise PerceptionError("no usable red target component")
    candidates.reverse()
    if (len(candidates) > 1 and
            candidates[1][0] >= candidates[0][0] * float(ambiguity_ratio)):
        raise PerceptionError("red target observation is ambiguous")
    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    mask[labels == candidates[0][1]] = 255
    return mask


def backproject_mask(
        mask, depth_m, camera_k, min_depth=0.15, max_depth=4.0):
    """Back-project finite masked depth pixels in ROS optical coordinates."""
    binary = np.asarray(mask).astype(bool)
    depth = np.asarray(depth_m, dtype=np.float32)
    if binary.ndim != 2 or depth.shape != binary.shape:
        raise PerceptionError("mask and depth dimensions differ")
    lower = float(min_depth)
    upper = float(max_depth)
    if (not math.isfinite(lower) or not math.isfinite(upper) or
            lower <= 0.0 or lower >= upper):
        raise PerceptionError("depth bounds are invalid")
    calibration = _camera_matrix(camera_k)
    finite = binary & np.isfinite(depth)
    valid = np.zeros(depth.shape, dtype=bool)
    valid[finite] = (
        (depth[finite] > lower) & (depth[finite] < upper))
    rows, columns = np.nonzero(valid)
    if not len(rows):
        raise PerceptionError("target mask has no valid depth")
    z = depth[rows, columns].astype(np.float64)
    x = ((columns.astype(np.float64) - calibration[0, 2]) * z /
         calibration[0, 0])
    y = ((rows.astype(np.float64) - calibration[1, 2]) * z /
         calibration[1, 1])
    return np.column_stack((x, y, z))


def canonical_yaw(value):
    return (float(value) + math.pi / 2.0) % math.pi - math.pi / 2.0


def yaw_error_mod_pi(first, second):
    return abs(canonical_yaw(float(first) - float(second)))


def estimate_target_pose(
        points, target_height, top_surface_tolerance=0.012):
    """Estimate cuboid center xyz and undirected long-axis yaw."""
    cloud = np.asarray(points, dtype=np.float64)
    height = float(target_height)
    tolerance = float(top_surface_tolerance)
    if (cloud.ndim != 2 or cloud.shape[1:] != (3,) or
            not math.isfinite(height) or height <= 0.0 or
            not math.isfinite(tolerance) or tolerance <= 0.0):
        raise PerceptionError("target pose inputs are invalid")
    cloud = cloud[np.isfinite(cloud).all(axis=1)]
    if len(cloud) < 3:
        raise PerceptionError("too few finite target points")
    top_level = float(np.percentile(cloud[:, 2], 95.0))
    top = cloud[cloud[:, 2] >= top_level - tolerance]
    if len(top) < 3:
        raise PerceptionError("too few target top-surface points")
    mean_xy = np.mean(top[:, :2], axis=0)
    centered = top[:, :2] - mean_xy
    covariance = centered.T.dot(centered) / float(len(centered))
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    if not np.isfinite(eigenvalues).all() or eigenvalues[-1] <= 1e-12:
        raise PerceptionError("target top surface is degenerate")
    long_axis = eigenvectors[:, int(np.argmax(eigenvalues))]
    short_axis = np.array((-long_axis[1], long_axis[0]))
    long_projection = centered.dot(long_axis)
    short_projection = centered.dot(short_axis)
    center_xy = (
        mean_xy +
        0.5 * (long_projection.min() + long_projection.max()) * long_axis +
        0.5 * (short_projection.min() + short_projection.max()) * short_axis)
    center_z = float(np.median(top[:, 2])) - 0.5 * height
    yaw = canonical_yaw(math.atan2(long_axis[1], long_axis[0]))
    pose = np.array((center_xy[0], center_xy[1], center_z, yaw))
    if not np.isfinite(pose).all():
        raise PerceptionError("target pose is non-finite")
    return pose


def fuse_pose_samples(
        samples, max_position_spread=0.04, max_yaw_spread=0.10):
    """Fuse a short stable pose window, respecting pi-periodic target yaw."""
    values = np.asarray(tuple(samples), dtype=np.float64)
    position_limit = float(max_position_spread)
    yaw_limit = float(max_yaw_spread)
    if (values.ndim != 2 or values.shape[1:] != (4,) or len(values) < 2 or
            not np.isfinite(values).all() or
            not math.isfinite(position_limit) or position_limit <= 0.0 or
            not math.isfinite(yaw_limit) or yaw_limit <= 0.0):
        raise PerceptionError("pose fusion inputs are invalid")
    position = np.mean(values[:, :3], axis=0)
    position_spread = float(np.max(np.linalg.norm(
        values[:, :3] - position, axis=1)))
    doubled = values[:, 3] * 2.0
    yaw = 0.5 * math.atan2(
        float(np.mean(np.sin(doubled))),
        float(np.mean(np.cos(doubled))))
    yaw = canonical_yaw(yaw)
    yaw_spread = max(yaw_error_mod_pi(value, yaw) for value in values[:, 3])
    if position_spread > position_limit or yaw_spread > yaw_limit:
        raise PerceptionError("target pose window is unstable")
    return np.array((position[0], position[1], position[2], yaw))
