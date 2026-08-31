"""Geometry-only 4DoF box pose estimation."""
from __future__ import division

import math

import numpy as np


def normalize_yaw_mod_pi(yaw):
    """Return the canonical representative of an undirected long axis."""
    return (float(yaw) + math.pi / 2.0) % math.pi - math.pi / 2.0


def yaw_error_mod_pi(estimate, reference):
    """Smallest angular distance for axes equivalent under a pi rotation."""
    return abs(normalize_yaw_mod_pi(float(estimate) - float(reference)))


def _dimension_aware_midpoint(projection, pixels, dimension, image_size,
                              border_margin):
    observed_midpoint = 0.5 * (projection.min() + projection.max())
    observed_span = projection.max() - projection.min()
    if observed_span >= 0.98 * float(dimension):
        return observed_midpoint
    if pixels is None or image_size is None:
        raise ValueError('partial box extent has no image-boundary evidence')

    width, height = image_size
    margin = float(border_margin)
    at_border = ((pixels[:, 0] <= margin) |
                 (pixels[:, 0] >= width - 1.0 - margin) |
                 (pixels[:, 1] <= margin) |
                 (pixels[:, 1] >= height - 1.0 - margin))
    endpoint_band = max(0.005, 0.03 * observed_span)
    low = projection <= projection.min() + endpoint_band
    high = projection >= projection.max() - endpoint_band
    low_score = at_border[low].mean() if np.any(low) else 0.0
    high_score = at_border[high].mean() if np.any(high) else 0.0
    if high_score >= 0.10 and low_score < 0.05:
        return projection.min() + float(dimension) / 2.0
    if low_score >= 0.10 and high_score < 0.05:
        return projection.max() - float(dimension) / 2.0
    raise ValueError('partial box extent has ambiguous missing direction')


def estimate_box_4dof(points, length, width, height,
                      top_surface_tolerance=0.008, min_top_points=100,
                      image_points=None, image_size=None,
                      image_border_margin=2.0, min_pca_ratio=1.5,
                      min_dimension_coverage=0.5,
                      max_dimension_ratio=1.25):
    """Estimate [x, y, z, yaw] from a horizontal rectangular box point set.

    Only the upper surface is used for XY PCA/OBB fitting.  The geometric Z
    center is recovered from that surface using the known box height.
    """
    cloud = np.asarray(points, dtype=np.float64)
    if cloud.ndim != 2 or cloud.shape[1] != 3:
        raise ValueError('points must have shape Nx3')
    finite_mask = np.all(np.isfinite(cloud), axis=1)
    finite = cloud[finite_mask]
    finite_pixels = None
    if image_points is not None:
        pixels = np.asarray(image_points, dtype=np.float64)
        if pixels.shape != (cloud.shape[0], 2):
            raise ValueError('image_points must have shape Nx2')
        finite_pixels = pixels[finite_mask]
    if finite.shape[0] < int(min_top_points):
        raise ValueError('not enough finite points')
    if length <= width or width <= 0.0 or height <= 0.0:
        raise ValueError('expected positive dimensions with length > width')

    top_level = np.percentile(finite[:, 2], 95.0)
    top_mask = finite[:, 2] >= top_level - float(top_surface_tolerance)
    top = finite[top_mask]
    top_pixels = finite_pixels[top_mask] if finite_pixels is not None else None
    if top.shape[0] < int(min_top_points):
        raise ValueError('not enough top-surface points')

    xy_mean = top[:, :2].mean(axis=0)
    centered = top[:, :2] - xy_mean
    covariance = np.dot(centered.T, centered) / top.shape[0]
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    minor_variance = eigenvalues[0]
    major_variance = eigenvalues[-1]
    if (minor_variance <= 1e-10 or
            major_variance / minor_variance < float(min_pca_ratio)):
        raise ValueError('top surface has degenerate PCA geometry')
    major = eigenvectors[:, int(np.argmax(eigenvalues))]
    minor = np.array((-major[1], major[0]))

    major_projection = np.dot(centered, major)
    minor_projection = np.dot(centered, minor)
    major_span = major_projection.max() - major_projection.min()
    minor_span = minor_projection.max() - minor_projection.min()
    if (major_span < float(min_dimension_coverage) * length or
            minor_span < float(min_dimension_coverage) * width or
            major_span > float(max_dimension_ratio) * length or
            minor_span > float(max_dimension_ratio) * width):
        raise ValueError('observed top surface is inconsistent with box size')
    major_midpoint = _dimension_aware_midpoint(
        major_projection, top_pixels, length, image_size,
        image_border_margin)
    minor_midpoint = _dimension_aware_midpoint(
        minor_projection, top_pixels, width, image_size,
        image_border_margin)
    center_xy = (xy_mean + major_midpoint * major +
                 minor_midpoint * minor)

    # The point cloud observes the upper face, not the box centroid.
    center_z = np.median(top[:, 2]) - float(height) / 2.0
    yaw = normalize_yaw_mod_pi(math.atan2(major[1], major[0]))
    return np.array((center_xy[0], center_xy[1], center_z, yaw),
                    dtype=np.float64)
