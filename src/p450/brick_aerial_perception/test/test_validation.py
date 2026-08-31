#!/usr/bin/env python3

import math
import pathlib
import unittest

import yaml

from brick_aerial_perception.validation import summarize_results


PACKAGE = pathlib.Path(__file__).resolve().parents[1]


class ValidationTest(unittest.TestCase):
    def test_summary_reports_success_and_error_extrema(self):
        results = [
            {"success": True, "position_error": 0.02, "xy_error": 0.01,
             "z_error": 0.017, "yaw_error": math.radians(2.0)},
            {"success": True, "position_error": 0.04, "xy_error": 0.03,
             "z_error": 0.026, "yaw_error": math.radians(4.0)},
            {"success": False, "reason": "no_valid_pose"},
        ]
        summary = summarize_results(results, expected_count=3)
        self.assertEqual(summary["success_count"], 2)
        self.assertAlmostEqual(summary["success_rate"], 2.0 / 3.0)
        self.assertAlmostEqual(summary["position_error_m"]["mean"], 0.03)
        self.assertAlmostEqual(summary["position_error_m"]["max"], 0.04)
        self.assertAlmostEqual(summary["yaw_error_deg"]["max"], 4.0)

    def test_four_fixed_views_have_distinct_brick_validation_poses(self):
        viewpoints = yaml.safe_load((PACKAGE / "config" / "viewpoints.yaml").read_text())
        validation = yaml.safe_load(
            (PACKAGE / "config" / "validation_scenarios.yaml").read_text()
        )
        view_ids = [item["id"] for item in viewpoints["viewpoints"]]
        scenarios = validation["validation_scenarios"]
        self.assertEqual(view_ids, [item["viewpoint"] for item in scenarios])
        poses = {(tuple(item["brick_position"]), item["brick_yaw"])
                 for item in scenarios}
        self.assertEqual(len(poses), 4)

    def test_runtime_monitor_uses_gazebo_gt_and_world_pose(self):
        source = (PACKAGE / "scripts" / "m1_validation_monitor.py").read_text()
        for token in (
            "SetModelState", "m1_brick", "SAMPLING:", "PoseStamped",
            "world", "position_error", "yaw_error", "camera/color/image_raw",
            "camera/depth/image_raw", "source_tree_sha256", "git_head",
        ):
            self.assertIn(token, source)

    def test_launch_can_enable_validation_without_changing_default_demo(self):
        source = (PACKAGE / "launch" / "m1_aerial_perception.launch").read_text()
        self.assertIn('name="validation_enable" default="false"', source)
        self.assertIn('type="m1_validation_monitor.py"', source)
        self.assertIn('if="$(arg validation_enable)"', source)


if __name__ == "__main__":
    unittest.main()
