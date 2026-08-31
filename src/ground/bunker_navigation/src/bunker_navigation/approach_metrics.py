"""Metrics for known-brick approach generation and navigation trials."""
from __future__ import division

import math

from bunker_navigation.heading_gate import normalize_angle


def relative_brick_metrics(base_pose, brick_pose, arm_offset_x,
                           arm_offset_y, work_distance):
    dx = float(brick_pose[0]) - float(base_pose[0])
    dy = float(brick_pose[1]) - float(base_pose[1])
    yaw = float(base_pose[2])
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    base_forward = cosine * dx + sine * dy
    base_lateral = -sine * dx + cosine * dy
    arm_forward = base_forward - float(arm_offset_x)
    arm_lateral = base_lateral - float(arm_offset_y)
    desired_center_distance = math.hypot(
        float(arm_offset_x) + float(work_distance), float(arm_offset_y))
    center_distance = math.hypot(dx, dy)
    return {
        'center_distance': center_distance,
        'desired_center_distance': desired_center_distance,
        'distance_error': abs(center_distance - desired_center_distance),
        'facing_error': abs(normalize_angle(math.atan2(dy, dx) - yaw)),
        'arm_forward': arm_forward,
        'arm_lateral': arm_lateral,
        'arm_workspace_error': math.hypot(
            arm_forward - float(work_distance), arm_lateral),
    }


def _mean(values):
    return sum(values) / len(values) if values else None


def summarize_approach_trials(trials):
    valid = [item for item in trials if item['expect_generation']]
    rejection = [item for item in trials if not item['expect_generation']]
    generated = [item for item in valid if item['generation_success']]
    navigated = [item for item in valid if item['navigation_success']]
    safe_rejections = [item for item in rejection
                       if not item['generation_success']]
    distances = [float(item['distance_error']) for item in valid
                 if item.get('distance_error') is not None]
    facings = [float(item['facing_error']) for item in valid
               if item.get('facing_error') is not None]
    return {
        'trial_count': len(trials),
        'valid_trial_count': len(valid),
        'rejection_trial_count': len(rejection),
        'generation_success_count': len(generated),
        'generation_success_rate': (
            len(generated) / len(valid) if valid else 0.0),
        'navigation_success_count': len(navigated),
        'navigation_success_rate': (
            len(navigated) / len(valid) if valid else 0.0),
        'safe_rejection_count': len(safe_rejections),
        'safe_rejection_rate': (
            len(safe_rejections) / len(rejection) if rejection else 1.0),
        'distance_error_mean': _mean(distances),
        'distance_error_max': max(distances) if distances else None,
        'facing_error_mean': _mean(facings),
        'facing_error_max': max(facings) if facings else None,
        'worst_distance_case': (max(
            (item for item in valid if item.get('distance_error') is not None),
            key=lambda item: item['distance_error']) if distances else None),
        'worst_facing_case': (max(
            (item for item in valid if item.get('facing_error') is not None),
            key=lambda item: item['facing_error']) if facings else None),
        'slowest_case': (max(trials, key=lambda item: item['duration'])
                         if trials else None),
    }
