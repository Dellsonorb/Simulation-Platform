"""Pure NumPy geometry for registered RGB-D brick pose estimation."""

import math

import numpy as np


def _camera_matrix(value):
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.shape == (9,):
        matrix = matrix.reshape(3, 3)
    if matrix.shape != (3, 3):
        raise ValueError("camera matrix must be 3x3")
    if not np.isfinite(matrix).all() or matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
        raise ValueError("camera matrix is invalid")
    return matrix


def register_depth_to_color(depth_m, depth_k, color_k, output_shape,
                            rotation=None, translation=None):
    """Project a depth image into the color imager and z-buffer collisions.

    Points and extrinsics use ROS optical coordinates: x right, y down, z
    forward. Invalid output pixels are NaN so downstream code cannot confuse
    missing data with a real zero range.
    """
    depth = np.asarray(depth_m, dtype=np.float32)
    if depth.ndim != 2:
        raise ValueError("depth image must be two-dimensional")
    rows, cols = depth.shape
    out_rows, out_cols = tuple(int(item) for item in output_shape)
    kd = _camera_matrix(depth_k)
    kc = _camera_matrix(color_k)
    rot = np.eye(3) if rotation is None else np.asarray(rotation, dtype=np.float64)
    trans = np.zeros(3) if translation is None else np.asarray(translation, dtype=np.float64)
    if rot.shape != (3, 3) or trans.shape != (3,):
        raise ValueError("depth-to-color extrinsics must be R(3x3), t(3)")

    vv, uu = np.indices((rows, cols), dtype=np.float64)
    valid = np.isfinite(depth) & (depth > 0.0)
    z = depth[valid].astype(np.float64)
    if z.size == 0:
        return np.full((out_rows, out_cols), np.nan, dtype=np.float32)
    x = (uu[valid] - kd[0, 2]) * z / kd[0, 0]
    y = (vv[valid] - kd[1, 2]) * z / kd[1, 1]
    color_points = np.column_stack((x, y, z)).dot(rot.T) + trans
    z_color = color_points[:, 2]
    projectable = np.isfinite(color_points).all(axis=1) & (z_color > 0.0)
    color_points = color_points[projectable]
    z_color = z_color[projectable]
    u_color = np.rint(kc[0, 0] * color_points[:, 0] / z_color + kc[0, 2]).astype(np.int64)
    v_color = np.rint(kc[1, 1] * color_points[:, 1] / z_color + kc[1, 2]).astype(np.int64)
    inside = ((u_color >= 0) & (u_color < out_cols) &
              (v_color >= 0) & (v_color < out_rows))

    flat = np.full(out_rows * out_cols, np.inf, dtype=np.float64)
    indices = v_color[inside] * out_cols + u_color[inside]
    np.minimum.at(flat, indices, z_color[inside])
    flat[~np.isfinite(flat)] = np.nan
    return flat.reshape(out_rows, out_cols).astype(np.float32)


def backproject_mask(mask, aligned_depth_m, camera_k, min_depth=0.2, max_depth=8.0):
    """Back-project valid masked pixels into the camera optical frame."""
    binary = np.asarray(mask).astype(bool)
    depth = np.asarray(aligned_depth_m, dtype=np.float32)
    if binary.shape != depth.shape:
        raise ValueError("mask and aligned depth must have the same shape")
    k = _camera_matrix(camera_k)
    # NumPy can raise ``invalid`` while comparing NaN/Inf under strict global
    # floating-point settings, even when a finite mask is part of the same
    # vector expression. Only compare finite elements.
    finite = binary & np.isfinite(depth)
    valid = np.zeros(depth.shape, dtype=bool)
    valid[finite] = ((depth[finite] > float(min_depth))
                     & (depth[finite] < float(max_depth)))
    vv, uu = np.nonzero(valid)
    z = depth[vv, uu].astype(np.float64)
    x = (uu.astype(np.float64) - k[0, 2]) * z / k[0, 0]
    y = (vv.astype(np.float64) - k[1, 2]) * z / k[1, 1]
    return np.column_stack((x, y, z))


def canonical_yaw(yaw):
    """Canonicalize an undirected long-axis yaw to [-pi/2, pi/2)."""
    return (float(yaw) + math.pi / 2.0) % math.pi - math.pi / 2.0


def yaw_error_mod_pi(estimate, reference):
    """Smallest absolute error for a yaw whose axis is pi-periodic."""
    return abs(canonical_yaw(float(estimate) - float(reference)))


def estimate_brick_pose(points_world, brick_height, top_surface_tolerance=0.015):
    """Estimate center xyz and long-axis yaw from the upper brick surface.

    An aerial oblique view contains both top and side returns. Fitting all
    returns biases PCA toward the visible side, so first select a narrow band
    around the upper surface and fit a 2-D oriented bounding box there.
    """
    points = np.asarray(points_world, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape Nx3")
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < 3:
        raise ValueError("at least three finite points are required")
    top_level = float(np.percentile(points[:, 2], 95.0))
    top = points[points[:, 2] >= top_level - float(top_surface_tolerance)]
    if len(top) < 3:
        raise ValueError("at least three top-surface points are required")
    mean_xy = np.mean(top[:, :2], axis=0)
    centered = top[:, :2] - mean_xy
    covariance = centered.T.dot(centered) / float(len(centered))
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    long_axis = eigenvectors[:, int(np.argmax(eigenvalues))]
    short_axis = np.array([-long_axis[1], long_axis[0]])
    long_projection = centered.dot(long_axis)
    short_projection = centered.dot(short_axis)
    center_xy = (mean_xy
                 + 0.5 * (long_projection.min() + long_projection.max()) * long_axis
                 + 0.5 * (short_projection.min() + short_projection.max()) * short_axis)
    yaw = canonical_yaw(math.atan2(long_axis[1], long_axis[0]))
    top_z = float(np.median(top[:, 2]))
    center_z = top_z - 0.5 * float(brick_height)
    return np.array([center_xy[0], center_xy[1], center_z, yaw], dtype=np.float64)
