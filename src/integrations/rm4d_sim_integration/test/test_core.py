#!/usr/bin/env python3

import math
import sys
import unittest
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "src"))

from rm4d_sim_integration.core import (  # noqa: E402
    FROZEN_RM4D_COMMIT,
    IntegrationCore,
    require_external_revision,
)
from rm4d_sim_integration.geometry import (  # noqa: E402
    IntegrationError,
    PoseValues,
    quaternion_angle,
)


class RecordingAPI:
    def __init__(self, result):
        self.result = result
        self.request = None
        self.top_k = None

    def plan(self, request, top_k=None):
        self.request = request
        self.top_k = top_k
        return self.result


def candidate(candidate_id, x, score):
    return {
        "candidate_id": candidate_id,
        "bunker_x": x,
        "bunker_y": 0.25,
        "bunker_yaw": -0.2,
        "final_score": score,
    }


class IntegrationCoreTest(unittest.TestCase):
    def setUp(self):
        self.exact = PoseValues(
            (2.0, 0.0, 0.0794),
            (0.0, math.sin(math.pi / 4.0),
             0.0, math.cos(math.pi / 4.0)))

    def test_plan_passes_regularized_copy_and_preserves_rank(self):
        api = RecordingAPI({
            "status": "ok", "frame_id": "map",
            "candidates": [
                candidate("second", 2.5, 0.8),
                candidate("first", 2.7, 0.9),
            ],
        })
        planner = IntegrationCore(api)

        result = planner.plan(
            "map", self.exact, (3.0, 0.0, 0.1), "brick-001", 2)

        self.assertEqual(2, api.top_k)
        self.assertEqual("map", api.request["frame_id"])
        self.assertAlmostEqual(
            1e-6,
            quaternion_angle(
                self.exact.orientation, api.request["quaternion_xyzw"]),
            places=9)
        self.assertEqual(
            ["second", "first"],
            [item.candidate_id for item in result.candidates])

    def test_no_feasible_candidate_is_a_valid_empty_result(self):
        api = RecordingAPI({
            "status": "no_feasible_candidate", "frame_id": "map",
            "candidates": [],
        })

        result = IntegrationCore(api).plan(
            "map", self.exact, (3.0, 0.0, 0.0), "brick-001", 5)

        self.assertEqual("no_feasible_candidate", result.status)
        self.assertEqual((), result.candidates)

    def test_result_frame_and_status_must_match_the_public_contract(self):
        invalid_results = (
            {"status": "ok", "frame_id": "world", "candidates": []},
            {"status": "retry", "frame_id": "map", "candidates": []},
            {"status": "no_feasible_candidate", "frame_id": "map",
             "candidates": [candidate("unexpected", 1.0, 0.1)]},
        )
        for value in invalid_results:
            with self.subTest(value=value):
                with self.assertRaises(IntegrationError):
                    IntegrationCore(RecordingAPI(value)).plan(
                        "map", self.exact, (3.0, 0.0, 0.0),
                        "brick-001", 5)

    def test_top_k_must_be_positive(self):
        api = RecordingAPI({
            "status": "no_feasible_candidate", "frame_id": "map",
            "candidates": [],
        })
        for value in (0, -1, True, 1.5):
            with self.subTest(value=value):
                with self.assertRaisesRegex(IntegrationError, "top_k"):
                    IntegrationCore(api).plan(
                        "map", self.exact, (3.0, 0.0, 0.0),
                        "brick-001", value)

    def test_external_revision_must_match_the_frozen_commit_exactly(self):
        self.assertEqual(
            "e9d431299053f38a4a4319aed3dfeccc261b9fac",
            FROZEN_RM4D_COMMIT)
        require_external_revision(FROZEN_RM4D_COMMIT)
        with self.assertRaisesRegex(IntegrationError, "revision mismatch"):
            require_external_revision(
                "fb2a84535c4f4c764e17c44fb3f79b11d304e733")


if __name__ == "__main__":
    unittest.main()
