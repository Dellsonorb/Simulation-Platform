#!/usr/bin/env python3

import importlib.util
import math
from pathlib import Path
import unittest


MODULE = Path(__file__).resolve().parents[1] / (
    "src/air_ground_pick_demo/approach.py")


def load_module():
    spec = importlib.util.spec_from_file_location(
        "air_ground_standoff_test_target", str(MODULE))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ApproachTest(unittest.TestCase):
    def test_latched_position_history_does_not_replace_actual_arrival(self):
        approach = load_module()
        self.assertTrue(approach.arrival_within_tolerance((.04, 0, -.01), (0, 0, 0), .06, .08))
        self.assertFalse(approach.arrival_within_tolerance((.0986, 0, .07), (0, 0, 0), .06, .08))
        self.assertFalse(approach.arrival_within_tolerance((0, 0, .09), (0, 0, 0), .06, .08))
        self.assertTrue(approach.arrival_within_tolerance((0, 0, math.pi), (0, 0, -math.pi), .06, .08))

    def test_goal_stays_on_current_base_side_and_faces_target(self):
        approach = load_module()
        goal = approach.compute_standoff_goal(
            current_xy=(3.0, 0.0), target_xy=(2.0, 0.0), standoff=0.82)
        self.assertAlmostEqual(2.82, goal.x)
        self.assertAlmostEqual(0.0, goal.y)
        self.assertAlmostEqual(math.pi, abs(goal.yaw))
        target_to_goal = (goal.x - 2.0, goal.y)
        target_to_current = (1.0, 0.0)
        self.assertGreater(sum(a * b for a, b in zip(
            target_to_goal, target_to_current)), 0.0)
        self.assertAlmostEqual(
            0.82, math.hypot(goal.x - 2.0, goal.y))

    def test_goal_is_symmetric_and_motion_decision_is_bounded(self):
        approach = load_module()
        goal = approach.compute_standoff_goal(
            current_xy=(0.0, 0.0), target_xy=(2.0, 0.0), standoff=0.82)
        self.assertAlmostEqual(1.18, goal.x)
        self.assertAlmostEqual(0.0, goal.y)
        self.assertAlmostEqual(0.0, goal.yaw)
        self.assertTrue(approach.motion_required(
            (0.0, 0.0), (goal.x, goal.y), tolerance=0.05))
        self.assertFalse(approach.motion_required(
            (1.16, 0.0), (goal.x, goal.y), tolerance=0.05))
        self.assertAlmostEqual(
            0.75, approach.travel_distance((3.5, 0.0), (2.75, 0.0)))

    def test_staged_goal_positions_before_turning_to_target(self):
        approach = load_module()
        positioning, final = approach.compute_staged_standoff_goals(
            current_pose=(3.0, 0.0, 0.2),
            target_xy=(2.0, 0.0), standoff=0.82)

        self.assertEqual((final.x, final.y),
                         (positioning.x, positioning.y))
        self.assertAlmostEqual(0.2, positioning.yaw)
        self.assertAlmostEqual(math.pi, abs(final.yaw))

        heading = approach.compute_heading_goal(
            current_pose=(2.84, 0.01, 0.2), target_xy=(2.0, 0.0))
        self.assertEqual((2.84, 0.01), (heading.x, heading.y))
        self.assertAlmostEqual(
            math.atan2(-0.01, -0.84), heading.yaw)

    def test_staged_candidate_finishes_at_exact_rm4d_pose(self):
        approach = load_module()

        positioning, final = approach.compute_staged_candidate_goals(
            (3.0, 0.0, 0.2), (2.575, 0.375, -0.4))

        expected = approach.GoalGeometry(2.575, 0.375, -0.4)
        self.assertEqual(expected, positioning)
        self.assertEqual(expected, final)

    def test_nonfinite_rm4d_candidate_is_rejected(self):
        approach = load_module()
        with self.assertRaises(approach.ApproachError):
            approach.compute_staged_candidate_goals(
                (3.0, 0.0, 0.0), (2.5, float("nan"), 0.0))

    def test_degenerate_or_nonfinite_goal_is_rejected(self):
        approach = load_module()
        for current, target, standoff in (
                ((2.0, 0.0), (2.0, 0.0), 0.82),
                ((float("nan"), 0.0), (2.0, 0.0), 0.82),
                ((3.0, 0.0), (2.0, 0.0), 0.0)):
            with self.subTest(current=current, target=target):
                with self.assertRaises(approach.ApproachError):
                    approach.compute_standoff_goal(
                        current, target, standoff)


if __name__ == "__main__":
    unittest.main()
