#!/usr/bin/env python3

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[2]
ORCHESTRATOR = PACKAGE / "scripts/run_air_ground_pick_demo.py"
CONFIG = PACKAGE / "config/demo.yaml"
LAUNCH = PACKAGE / "launch/air_ground_pick_demo.launch"
RM4D_LAUNCH = ROOT / (
    "src/integrations/rm4d_sim_integration/launch/"
    "rm4d_air_ground_pick_demo.launch")
SIM_COMPOSITION = (
    ROOT / "src/platform/sim_platform_bringup/launch/air_ground_standalone.launch")


class OrchestratorContractTest(unittest.TestCase):
    def test_common_orchestrator_uses_only_robot_facing_actions(self):
        source = ORCHESTRATOR.read_text(encoding="utf-8")
        for required in (
                "FlightCommandAction", "FlightCommandGoal",
                "MoveBaseAction", "MoveBaseGoal", "Trigger",
                "self._flight_action", "self._ground_navigation_action",
                "self._ground_stop_service", "grasp_confirmed"):
            self.assertIn(required, source)
        for command in ("TAKEOFF", "FLY_TO", "HOVER", "LAND"):
            self.assertIn("FlightCommandGoal.%s" % command, source)
        lowered = source.lower()
        for forbidden in (
                "gazebo_msgs", "contactsstate", "uavcommand", "uavsetup",
                "uavcontrolstate", "laserscan", "twist", "/gazebo/",
                "/ground/cmd_vel", "/ground/nav_cmd_vel",
                "/uav1/prometheus/command", "/uav1/prometheus/setup",
                "bilateral", "teleport", "gazebo_ros_link_attacher"):
            self.assertNotIn(forbidden, lowered)
        # Planning-scene payload modeling is permitted; physical simulator
        # attachment and backend state access remain forbidden above.
        for required in ("GetPlanningScene", "ApplyPlanningScene", "GetStateValidity"):
            self.assertIn(required, source)

    def test_config_names_map_and_common_runtime_endpoints(self):
        config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
        self.assertEqual("map", config["map_frame"])
        self.assertEqual("/uav1/runtime/flight", config["flight_action"])
        self.assertEqual(
            "/ground/move_base", config["ground_navigation_action"])
        self.assertEqual(
            "/ground/runtime/stop", config["ground_stop_service"])
        self.assertEqual(
            "/ground/gripper/grasp_confirmed",
            config["grasp_confirmed_topic"])
        self.assertEqual("standoff", config["placement_mode"])
        self.assertEqual(
            "/rm4d/plan_base_placement", config["rm4d_service"])
        self.assertGreater(config["rm4d_top_k"], 0)
        self.assertEqual(6, config["rm4d_pregrasp_plan_attempts"])
        self.assertNotIn("rm4d_alignment_distance", config)
        for removed in (
                "uav_setup_topic", "uav_command_topic",
                "uav_control_state_topic", "ground_cmd_topic",
                "ground_scan_topic", "world_frame", "ground_timeout"):
            self.assertNotIn(removed, config)

    def test_sim_composition_starts_common_flight_and_navigation_layers(self):
        demo_includes = [
            item.get("file")
            for item in ET.parse(str(LAUNCH)).getroot().findall("include")]
        self.assertFalse(any(
            "p450_flight_facade.launch" in item for item in demo_includes))
        self.assertFalse(any(
            "ground_navigation.launch" in item for item in demo_includes))
        sim_includes = [
            item.get("file")
            for item in ET.parse(str(SIM_COMPOSITION)).getroot().findall(
                "include")]
        self.assertEqual(1, sum(
            "p450_flight_facade.launch" in item for item in sim_includes))
        self.assertEqual(1, sum(
            "ground_navigation.launch" in item for item in sim_includes))
        for observer in ("air_observer.yaml", "ground_observer.yaml"):
            config = yaml.safe_load((PACKAGE / "config" / observer).read_text(
                encoding="utf-8"))
            self.assertEqual("map", config["target_frame"])

    def test_rm4d_mode_preserves_top_one_and_uses_exact_pregrasp(self):
        source = ORCHESTRATOR.read_text(encoding="utf-8")
        selection = source.split(
            "    def _select_rm4d_candidate(", 1)[1].split(
            "    def _approach_ground_rm4d(", 1)[0]
        self.assertIn("generate_top_down_grasp", selection)
        self.assertIn("PlanBasePlacementRequest", source)
        self.assertIn("_rm4d_request_type", selection)
        self.assertIn("response.candidates.poses[0]", selection)
        self.assertIn("response.candidate_ids[0]", selection)
        self.assertIn("response.scores[0]", selection)

        approach = source.split(
            "    def _approach_ground_rm4d(", 1)[1].split(
            "    def _approach_ground_standoff(", 1)[0]
        self.assertIn("compute_staged_candidate_goals", approach)
        self.assertIn("RM4D_CANDIDATES", approach)
        self.assertIn('response.status == "no_feasible_candidate"', source)
        self.assertIn(
            "final_goal, \"RM4D top-1 navigation\"", approach)
        self.assertNotIn("transit heading", approach)
        self.assertNotIn("alignment position", approach)
        self.assertNotIn("compute_standoff_goal", approach)
        self.assertNotIn("compute_heading_goal", approach)

        observation = source.split(
            "    def _observe_ground_target_rm4d(", 1)[1].split(
            "    def _observe_ground_target_standoff(", 1)[0]
        self.assertIn("generate_top_down_grasp", observation)
        self.assertIn("generated.pregrasp", observation)
        self.assertIn("generated.grasp", observation)
        self.assertIn("rospy.Time.now()", observation)
        self.assertIn(
            "_execute_pregrasp(\n            target_facing_pregrasp, "
            "target_facing_grasp)", observation)
        self.assertNotIn("_move_to_ground_observation", observation)
        self.assertNotIn("regularize", observation)

        pregrasp = source.split(
            "    def _continuation_from_plan(", 1)[1].split(
            "    def _execute_cartesian(", 1)[0]
        self.assertIn("GetCartesianPathRequest", source)
        self.assertIn("self._rm4d_pregrasp_plan_attempts", pregrasp)
        self.assertIn("continuation.pose", pregrasp)
        self.assertIn("response.fraction", pregrasp)
        self.assertIn("self._cartesian_min_fraction", pregrasp)

    def test_rm4d_composition_starts_adapter_and_selects_mode(self):
        root = ET.parse(str(RM4D_LAUNCH)).getroot()
        includes = root.findall("include")
        self.assertTrue(any(
            "rm4d_adapter.launch" in item.get("file", "")
            for item in includes))
        demo = next(
            item for item in includes
            if "air_ground_pick_demo.launch" in item.get("file", ""))
        arguments = {
            item.get("name"): item.get("value") for item in demo.findall("arg")}
        self.assertEqual("rm4d", arguments["placement_mode"])


if __name__ == "__main__":
    unittest.main()
