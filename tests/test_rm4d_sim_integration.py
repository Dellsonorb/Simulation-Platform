#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src/integrations/rm4d_sim_integration"
DEMO_LAUNCH = PACKAGE / "launch/rm4d_air_ground_pick_demo.launch"
SMOKE = ROOT / "scripts/smoke_rm4d_air_ground_pick_demo.bash"
README = ROOT / "README.md"
sys.path.insert(0, str(PACKAGE / "src"))


class Rm4dIntegrationRepositoryContractTest(unittest.TestCase):
    def test_package_exists_only_as_a_thin_integration(self):
        self.assertTrue((PACKAGE / "package.xml").is_file())
        self.assertTrue((PACKAGE / "CMakeLists.txt").is_file())
        self.assertFalse((PACKAGE / "src/rm4d").exists())

    def test_public_geometry_constant_is_frozen(self):
        from rm4d_sim_integration.geometry import (
            RM4D_LOCAL_Y_REGULARIZATION_RAD,
        )

        self.assertEqual(1e-6, RM4D_LOCAL_Y_REGULARIZATION_RAD)

    def test_bounded_smoke_requires_external_rm4d_and_natural_lift(self):
        self.assertTrue(SMOKE.is_file())
        self.assertTrue(SMOKE.stat().st_mode & 0o111)
        source = SMOKE.read_text(encoding="utf-8")
        for required in (
                "RM4D_ROOT", "RM4D_PYTHON", "RM4D_MAP",
                "rm4d_air_ground_pick_demo.launch", "/usr/bin/timeout",
                "RM4D_CANDIDATES", "GROUND_STOPPED", "LIFT",
                "RM4D_SIM_INTEGRATION_READY"):
            self.assertIn(required, source)
        for forbidden in (
                "/gazebo/model_states", "SetModelState", "teleport",
                "attach", "epsilon"):
            self.assertNotIn(forbidden.lower(), source.lower())

    def test_readme_documents_frozen_external_and_rviz_path(self):
        text = README.read_text(encoding="utf-8")
        for required in (
                "rm4d-aubo-baseline-v1",
                "e9d431299053f38a4a4319aed3dfeccc261b9fac",
                "77279fafaf61d5c92cf303a644cd9613457c85e0d197c66c4487c30d135db3b4",
                "Local-Y +1e-6 rad",
                "rm4d_candidates.rviz",
                "smoke_rm4d_air_ground_pick_demo.bash"):
            self.assertIn(required, text)

    def test_rm4d_demo_starts_at_the_clear_scene_parking_pose_facing_target(self):
        text = DEMO_LAUNCH.read_text(encoding="utf-8")
        for required in (
                '<arg name="bunker_x" default="3.0" />',
                '<arg name="bunker_y" default="0.0" />',
                '<arg name="bunker_z" default="0.36" />',
                '<arg name="bunker_yaw" default="3.141592653589793" />',
                '<arg name="bunker_x" value="$(arg bunker_x)" />',
                '<arg name="bunker_y" value="$(arg bunker_y)" />',
                '<arg name="bunker_z" value="$(arg bunker_z)" />',
                '<arg name="bunker_yaw" value="$(arg bunker_yaw)" />'):
            self.assertIn(required, text)


if __name__ == "__main__":
    unittest.main()
