"""Deterministic geometry for placing BUNKER around a known brick pose."""
from __future__ import division

import math
from collections import namedtuple

from bunker_navigation.costmap_sweep import (_footprint_cost,
                                             _polygons_overlap,
                                             _transform_footprint)
from bunker_navigation.heading_gate import normalize_angle


Candidate = namedtuple('Candidate', 'identifier x y yaw priority')


def _is_finite(value):
    value = float(value)
    return not math.isnan(value) and not math.isinf(value)


class ApproachParameters(namedtuple(
        '_ApproachParameters',
        'work_distance arm_offset_x arm_offset_y reach_min reach_max '
        'lateral_limit')):
    __slots__ = ()

    def validate(self):
        values = tuple(float(value) for value in self)
        if not all(_is_finite(value) for value in values):
            raise ValueError('approach parameters must be finite')
        if self.reach_min <= 0.0 or self.reach_max < self.reach_min:
            raise ValueError('invalid AUBO reach interval')
        if not self.reach_min <= self.work_distance <= self.reach_max:
            raise ValueError('work_distance is outside verified AUBO reach')
        if self.lateral_limit < 0.0 or abs(self.arm_offset_y) > 1.0:
            raise ValueError('invalid lateral workspace parameters')


def _finite_pose(pose, name):
    if len(pose) != 3 or not all(_is_finite(value)
                                 for value in pose):
        raise ValueError('{} pose must contain three finite values'.format(
            name))


def _candidate(identifier, brick, heading, parameters, priority):
    heading = normalize_angle(heading)
    forward = parameters.arm_offset_x + parameters.work_distance
    lateral = parameters.arm_offset_y
    cosine = math.cos(heading)
    sine = math.sin(heading)
    return Candidate(
        identifier=identifier,
        x=float(brick[0]) - cosine * forward + sine * lateral,
        y=float(brick[1]) - sine * forward - cosine * lateral,
        yaw=heading,
        priority=priority)


def generate_candidates(brick_pose, current_pose, parameters,
                        duplicate_tolerance=1e-6):
    """Return direct and brick-axis candidates, all facing the brick."""
    _finite_pose(brick_pose, 'brick')
    _finite_pose(current_pose, 'current')
    parameters.validate()
    dx = float(brick_pose[0]) - float(current_pose[0])
    dy = float(brick_pose[1]) - float(current_pose[1])
    if math.hypot(dx, dy) <= 1e-6:
        direct = float(current_pose[2])
    else:
        direct = math.atan2(dy, dx)
    brick_yaw = float(brick_pose[2])
    headings = (
        ('direct', direct),
        ('brick_long_positive', brick_yaw),
        ('brick_long_negative', brick_yaw + math.pi),
        ('brick_short_positive', brick_yaw + math.pi / 2.0),
        ('brick_short_negative', brick_yaw - math.pi / 2.0),
    )
    candidates = []
    accepted_headings = []
    for priority, (identifier, heading) in enumerate(headings):
        heading = normalize_angle(heading)
        if any(abs(normalize_angle(heading - accepted)) <=
               duplicate_tolerance for accepted in accepted_headings):
            continue
        candidates.append(_candidate(
            identifier, brick_pose, heading, parameters, priority))
        accepted_headings.append(heading)
    return candidates


def candidate_is_clear(grid, footprint, candidate, lethal_threshold=100):
    """Check the full padded BUNKER footprint at one candidate pose."""
    if len(footprint) < 3:
        raise ValueError('footprint requires at least three points')
    vertices = _transform_footprint(
        footprint, candidate.x, candidate.y, candidate.yaw)
    reason = _footprint_cost(grid, vertices, int(lethal_threshold))
    return reason == 'CLEAR', reason


def candidate_bearing_is_supported(candidate, current_pose,
                                   direct_drive_bearing_limit,
                                   heading_gate_entry_bearing):
    """Avoid the lateral gap between stable DWA and heading-gate entry."""
    dx = float(candidate.x) - float(current_pose[0])
    dy = float(candidate.y) - float(current_pose[1])
    bearing = abs(normalize_angle(
        math.atan2(dy, dx) - float(current_pose[2])))
    return (bearing <= float(direct_drive_bearing_limit) or
            bearing >= float(heading_gate_entry_bearing))


def costmap_is_ready(costmap_received_at, now, input_received_at,
                     max_age, settle_duration):
    """Require a fresh costmap update after the input settling horizon."""
    values = (now, input_received_at, max_age, settle_duration)
    if not all(_is_finite(value) for value in values) or \
            float(max_age) < 0.0 or float(settle_duration) < 0.0:
        raise ValueError('costmap timing must be finite and non-negative')
    if costmap_received_at is None or not _is_finite(costmap_received_at):
        return False
    ready_at = float(input_received_at) + float(settle_duration)
    age = float(now) - float(costmap_received_at)
    return (float(now) >= ready_at and
            float(costmap_received_at) >= ready_at and
            0.0 <= age <= float(max_age))


def _target_polygon(target_pose, length, width, padding):
    half_length = float(length) * 0.5 + float(padding)
    half_width = float(width) * 0.5 + float(padding)
    local = ((-half_length, -half_width), (-half_length, half_width),
             (half_length, half_width), (half_length, -half_width))
    return _transform_footprint(
        local, float(target_pose[0]), float(target_pose[1]),
        float(target_pose[2]))


def _interpolated_path(path, linear_step, angular_step):
    result = [tuple(float(value) for value in path[0])]
    for start, end in zip(path[:-1], path[1:]):
        start = tuple(float(value) for value in start)
        end = tuple(float(value) for value in end)
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        rotation = normalize_angle(end[2] - start[2])
        count = max(1, int(math.ceil(max(
            distance / float(linear_step),
            abs(rotation) / float(angular_step)))))
        for index in range(1, count + 1):
            ratio = index / float(count)
            result.append((
                start[0] + (end[0] - start[0]) * ratio,
                start[1] + (end[1] - start[1]) * ratio,
                normalize_angle(start[2] + rotation * ratio),
            ))
    return result


def path_sweep_avoids_target(path, footprint, target_pose,
                             brick_length, brick_width, target_padding,
                             linear_step, angular_step):
    """Protect a known low-profile target throughout a planned base path."""
    if not path or len(footprint) < 3:
        raise ValueError('path and footprint geometry must be non-empty')
    poses = tuple(tuple(item) for item in path)
    if (any(len(item) != 3 for item in poses) or len(target_pose) != 3 or
            any(len(item) != 2 for item in footprint)):
        raise ValueError('path, target and footprint geometry dimensions invalid')
    values = [value for item in poses for value in item]
    values.extend(target_pose)
    values.extend(value for item in footprint for value in item)
    if not all(_is_finite(value) for value in values):
        raise ValueError('path and target geometry must be finite')
    dimensions = (float(brick_length), float(brick_width),
                  float(linear_step), float(angular_step))
    if any(value <= 0.0 for value in dimensions) or \
            float(target_padding) < 0.0 or \
            not _is_finite(target_padding):
        raise ValueError('target dimensions and sweep steps must be positive')
    target = _target_polygon(
        target_pose, brick_length, brick_width, target_padding)
    checked = 0
    for x, y, yaw in _interpolated_path(
            poses, linear_step, angular_step):
        checked += 1
        robot = _transform_footprint(footprint, x, y, yaw)
        if _polygons_overlap(robot, target):
            return False, 'TARGET_SWEEP_COLLISION', checked
    return True, 'CLEAR', checked
