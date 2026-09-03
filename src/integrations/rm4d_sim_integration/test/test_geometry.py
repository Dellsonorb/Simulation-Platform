#!/usr/bin/env python3

import inspect
import math
import sys
import unittest
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "src"))

from rm4d_sim_integration.geometry import (  # noqa: E402
    IntegrationError,
    PoseValues,
    build_rm4d_request,
    ordered_candidates,
    quaternion_angle,
    quaternion_from_yaw,
    regularize_for_rm4d,
    yaw_from_quaternion,
)


class FrozenRegularizationTest(unittest.TestCase):
    def setUp(self):
        self.exact = PoseValues(
            position=(2.0, 0.0, 0.0794),
            orientation=(0.0, math.sin(math.pi / 4.0),
                         0.0, math.cos(math.pi / 4.0)))

    def test_regularization_is_fixed_local_y_and_keeps_exact_pose_immutable(self):
        original = tuple(self.exact.orientation)

        query = regularize_for_rm4d(self.exact)

        self.assertEqual((2.0, 0.0, 0.0794), query.position)
        self.assertEqual(original, self.exact.orientation)
        self.assertAlmostEqual(
            1e-6, quaternion_angle(self.exact.orientation,
                                   query.orientation), places=9)

    def test_regularization_has_no_runtime_epsilon_parameter(self):
        self.assertEqual(
            ["exact_pose"],
            list(inspect.signature(regularize_for_rm4d).parameters))

    def test_request_is_map_only_and_contains_current_bunker_se2(self):
        request = build_rm4d_request(
            "map", self.exact, (3.0, -0.1, 0.2), "brick-001")

        self.assertEqual("map", request["frame_id"])
        self.assertEqual("brick-001", request["grasp_id"])
        self.assertEqual([2.0, 0.0, 0.0794], request["position_xyz"])
        self.assertEqual(
            {"x": 3.0, "y": -0.1, "yaw": 0.2},
            request["current_bunker_pose"])
        self.assertAlmostEqual(
            1e-6,
            quaternion_angle(
                self.exact.orientation, request["quaternion_xyzw"]),
            places=9)

    def test_request_rejects_any_non_map_frame(self):
        for frame in ("world", "odom", "", None):
            with self.subTest(frame=frame):
                with self.assertRaisesRegex(IntegrationError, "must be map"):
                    build_rm4d_request(
                        frame, self.exact, (3.0, 0.0, 0.0), "brick-001")

    def test_candidate_conversion_preserves_external_rank_order(self):
        result = {
            "candidates": [
                {"candidate_id": "candidate-000162", "bunker_x": 2.57,
                 "bunker_y": 0.37, "bunker_yaw": 0.0,
                 "final_score": 0.93},
                {"candidate_id": "candidate-000013", "bunker_x": 2.75,
                 "bunker_y": -0.11, "bunker_yaw": 3.0,
                 "final_score": 0.91},
            ]
        }

        candidates = ordered_candidates(result)

        self.assertEqual(
            ["candidate-000162", "candidate-000013"],
            [candidate.candidate_id for candidate in candidates])
        self.assertEqual((2.57, 0.37, 0.0), candidates[0].pose)

    def test_planar_quaternion_round_trip_uses_xyzw(self):
        for yaw in (-math.pi, -1.2, 0.0, 0.8, math.pi):
            with self.subTest(yaw=yaw):
                recovered = yaw_from_quaternion(quaternion_from_yaw(yaw))
                error = (recovered - yaw + math.pi) % (2.0 * math.pi) - math.pi
                self.assertAlmostEqual(0.0, error, places=12)


if __name__ == "__main__":
    unittest.main()
