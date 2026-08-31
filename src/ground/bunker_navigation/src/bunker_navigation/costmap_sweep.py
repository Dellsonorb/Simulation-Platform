"""Pure footprint collision checks for an in-place heading sweep."""
from __future__ import division

import math

from bunker_navigation.heading_gate import normalize_angle


class CostmapGrid(object):
    def __init__(self, width, height, resolution, origin_x, origin_y, data):
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.data = list(data)
        if (self.width <= 0 or self.height <= 0 or self.resolution <= 0.0 or
                len(self.data) != self.width * self.height):
            raise ValueError('invalid costmap dimensions or data length')


class SweepResult(object):
    def __init__(self, clear, reason, checked_poses, collision_yaw=None):
        self.clear = bool(clear)
        self.reason = str(reason)
        self.checked_poses = int(checked_poses)
        self.collision_yaw = collision_yaw


def pad_footprint(footprint, padding):
    """Apply the same axis-wise padding convention as costmap_2d."""
    padding = float(padding)
    return [(x + (padding if x > 0.0 else -padding if x < 0.0 else 0.0),
             y + (padding if y > 0.0 else -padding if y < 0.0 else 0.0))
            for x, y in footprint]


def _transform_footprint(footprint, x, y, yaw):
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return [(x + cosine * px - sine * py,
             y + sine * px + cosine * py) for px, py in footprint]


def _project(vertices, axis):
    values = [point[0] * axis[0] + point[1] * axis[1]
              for point in vertices]
    return min(values), max(values)


def _polygons_overlap(first, second):
    axes = [(1.0, 0.0), (0.0, 1.0)]
    for polygon in (first, second):
        for index, start in enumerate(polygon):
            end = polygon[(index + 1) % len(polygon)]
            axes.append((-(end[1] - start[1]), end[0] - start[0]))
    for axis in axes:
        first_min, first_max = _project(first, axis)
        second_min, second_max = _project(second, axis)
        if first_max < second_min or second_max < first_min:
            return False
    return True


def _rotation_yaws(robot_pose, goal_xy, release_bearing, angular_step):
    x, y, yaw = robot_pose
    bearing = normalize_angle(
        math.atan2(goal_xy[1] - y, goal_xy[0] - x) - yaw)
    if abs(bearing) <= release_bearing:
        return [yaw]
    rotation = bearing - math.copysign(release_bearing, bearing)
    count = max(1, int(math.ceil(abs(rotation) / angular_step)))
    return [yaw + rotation * index / count for index in range(count + 1)]


def _footprint_cost(grid, vertices, lethal_threshold):
    minimum_x = min(point[0] for point in vertices)
    maximum_x = max(point[0] for point in vertices)
    minimum_y = min(point[1] for point in vertices)
    maximum_y = max(point[1] for point in vertices)
    map_maximum_x = grid.origin_x + grid.width * grid.resolution
    map_maximum_y = grid.origin_y + grid.height * grid.resolution
    if (minimum_x < grid.origin_x or minimum_y < grid.origin_y or
            maximum_x >= map_maximum_x or maximum_y >= map_maximum_y):
        return 'OUTSIDE_COSTMAP'
    minimum_mx = int(math.floor(
        (minimum_x - grid.origin_x) / grid.resolution))
    maximum_mx = int(math.floor(
        (maximum_x - grid.origin_x) / grid.resolution))
    minimum_my = int(math.floor(
        (minimum_y - grid.origin_y) / grid.resolution))
    maximum_my = int(math.floor(
        (maximum_y - grid.origin_y) / grid.resolution))
    for my in range(minimum_my, maximum_my + 1):
        cell_y = grid.origin_y + my * grid.resolution
        for mx in range(minimum_mx, maximum_mx + 1):
            value = grid.data[my * grid.width + mx]
            if value >= 0 and value < lethal_threshold:
                continue
            cell_x = grid.origin_x + mx * grid.resolution
            cell = ((cell_x, cell_y),
                    (cell_x, cell_y + grid.resolution),
                    (cell_x + grid.resolution, cell_y + grid.resolution),
                    (cell_x + grid.resolution, cell_y))
            if _polygons_overlap(vertices, cell):
                return 'UNKNOWN' if value < 0 else 'COLLISION'
    return 'CLEAR'


def check_rotation_sweep(grid, footprint, robot_pose, goal_xy,
                         release_bearing, angular_step,
                         lethal_threshold=100):
    """Check every footprint pose until the goal bearing reaches release."""
    if len(footprint) < 3:
        raise ValueError('footprint requires at least three points')
    release_bearing = float(release_bearing)
    angular_step = float(angular_step)
    if release_bearing <= 0.0 or angular_step <= 0.0:
        raise ValueError('release bearing and angular step must be positive')
    yaws = _rotation_yaws(robot_pose, goal_xy, release_bearing, angular_step)
    for index, yaw in enumerate(yaws):
        vertices = _transform_footprint(
            footprint, robot_pose[0], robot_pose[1], yaw)
        reason = _footprint_cost(grid, vertices, int(lethal_threshold))
        if reason != 'CLEAR':
            return SweepResult(False, reason, index + 1, yaw)
    return SweepResult(True, 'CLEAR', len(yaws))
