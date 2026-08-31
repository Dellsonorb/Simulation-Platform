from __future__ import division

import math


def _trajectory_from_plan(plan):
    if isinstance(plan, tuple):
        if len(plan) < 2 or not plan[0]:
            return None
        plan = plan[1]
    return getattr(plan, 'joint_trajectory', None)


def validated_observation_trajectory(plan, expected_names, target,
                                     endpoint_tolerance):
    trajectory = _trajectory_from_plan(plan)
    if trajectory is None or not getattr(trajectory, 'points', None):
        return None, 'planning_failed:empty_trajectory', None
    names = list(getattr(trajectory, 'joint_names', []))
    endpoint = list(trajectory.points[-1].positions)
    if len(names) != len(set(names)) or set(names) != set(expected_names) or \
            len(endpoint) != len(names):
        return None, 'planning_failed:joint_contract', None
    endpoint_by_name = dict(zip(names, endpoint))
    errors = [abs(endpoint_by_name[name] - value)
              for name, value in zip(expected_names, target)]
    if not all(not math.isnan(value) and not math.isinf(value)
               for value in errors):
        return None, 'planning_failed:nonfinite_endpoint', None
    if max(errors) > endpoint_tolerance:
        return None, 'planning_failed:endpoint_error', None
    duration = trajectory.points[-1].time_from_start.to_sec()
    if math.isnan(duration) or math.isinf(duration) or duration <= 0.0:
        return None, 'planning_failed:invalid_duration', None
    return trajectory, '', duration
