"""Pure image and depth operations used by the ROS node."""
from __future__ import division

import cv2
import numpy as np


def segment_hsv(rgb, hue_ranges, saturation_min, value_min,
                morphology_kernel, min_component_pixels):
    """Return a mono8 mask containing only the largest matching component."""
    if rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError('rgb must have shape HxWx3')
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    combined = np.zeros(rgb.shape[:2], dtype=np.uint8)
    for hue_low, hue_high in hue_ranges:
        candidate = cv2.inRange(
            hsv,
            np.array((hue_low, saturation_min, value_min), dtype=np.uint8),
            np.array((hue_high, 255, 255), dtype=np.uint8),
        )
        combined = cv2.bitwise_or(combined, candidate)

    kernel_size = max(1, int(morphology_kernel))
    if kernel_size > 1:
        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        combined = cv2.morphologyEx(combined, cv2.MORPH_OPEN, kernel)
        combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel)

    count, labels, stats, _ = cv2.connectedComponentsWithStats(combined, 8)
    if count <= 1:
        return np.zeros_like(combined)
    largest_label = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    if stats[largest_label, cv2.CC_STAT_AREA] < int(min_component_pixels):
        return np.zeros_like(combined)
    return np.where(labels == largest_label, 255, 0).astype(np.uint8)


def backproject_mask(depth, mask, camera_k, min_depth, max_depth,
                     pixel_stride=1):
    """Backproject valid masked depth pixels; return Nx3 points and Nx2 (u,v)."""
    if depth.shape != mask.shape:
        raise ValueError('depth and mask dimensions must match')
    if depth.ndim != 2:
        raise ValueError('depth must be a single-channel image')
    k = np.asarray(camera_k, dtype=np.float64).reshape((3, 3))
    fx, fy = k[0, 0], k[1, 1]
    cx, cy = k[0, 2], k[1, 2]
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError('camera focal lengths must be positive')
    stride = max(1, int(pixel_stride))

    with np.errstate(invalid='ignore'):
        valid = ((mask != 0) & np.isfinite(depth) &
                 (depth >= float(min_depth)) & (depth <= float(max_depth)))
    if stride > 1:
        sampling = np.zeros_like(valid)
        sampling[::stride, ::stride] = True
        valid &= sampling
    rows, columns = np.nonzero(valid)
    if rows.size == 0:
        return (np.empty((0, 3), dtype=np.float32),
                np.empty((0, 2), dtype=np.int32))

    z = depth[rows, columns].astype(np.float32)
    x = (columns.astype(np.float32) - cx) * z / fx
    y = (rows.astype(np.float32) - cy) * z / fy
    points = np.column_stack((x, y, z)).astype(np.float32)
    pixels = np.column_stack((columns, rows)).astype(np.int32)
    return points, pixels
