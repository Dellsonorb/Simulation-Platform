from __future__ import division

import unittest

from ground_pick_orchestrator.observation_planning import (
    validated_observation_trajectory)


class Duration(object):
    def __init__(self, value):
        self.value = value

    def to_sec(self):
        return self.value


class Point(object):
    def __init__(self, positions, duration=4.0):
        self.positions = positions
        self.time_from_start = Duration(duration)


class Trajectory(object):
    def __init__(self, names, points):
        self.joint_names = names
        self.points = points


class Plan(object):
    def __init__(self, trajectory):
        self.joint_trajectory = trajectory


class ObservationPlanningTest(unittest.TestCase):
    def test_accepts_complete_finite_plan_at_requested_endpoint(self):
        trajectory = Trajectory(['a', 'b'], [Point([0.2, -0.3])])
        result, reason, duration = validated_observation_trajectory(
            Plan(trajectory), ['a', 'b'], [0.2, -0.3], 0.01)
        self.assertIs(result, trajectory)
        self.assertEqual('', reason)
        self.assertEqual(4.0, duration)

    def test_rejects_failed_tuple_and_wrong_endpoint(self):
        result = validated_observation_trajectory(
            (False, Plan(Trajectory([], [])), 0.0, None),
            ['a'], [0.0], 0.01)
        self.assertEqual('planning_failed:empty_trajectory', result[1])
        result = validated_observation_trajectory(
            Plan(Trajectory(['a'], [Point([0.5])])),
            ['a'], [0.0], 0.01)
        self.assertEqual('planning_failed:endpoint_error', result[1])

    def test_rejects_joint_contract_and_invalid_duration(self):
        result = validated_observation_trajectory(
            Plan(Trajectory(['a', 'a'], [Point([0.0, 0.0])])),
            ['a', 'b'], [0.0, 0.0], 0.01)
        self.assertEqual('planning_failed:joint_contract', result[1])
        result = validated_observation_trajectory(
            Plan(Trajectory(['a'], [Point([0.0], 0.0)])),
            ['a'], [0.0], 0.01)
        self.assertEqual('planning_failed:invalid_duration', result[1])


if __name__ == '__main__':
    unittest.main()
