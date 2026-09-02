#!/usr/bin/env python3

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
ORCHESTRATOR = PACKAGE / "scripts/run_air_ground_pick_demo.py"
CONFIG = PACKAGE / "config/demo.yaml"
LAUNCH = PACKAGE / "launch/air_ground_pick_demo.launch"


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
                "uavcontrolstate", "laserscan", "twist", "world",
                "/ground/cmd_vel", "/ground/nav_cmd_vel",
                "/uav1/prometheus/command", "/uav1/prometheus/setup",
                "bilateral", "teleport", "attach"):
            self.assertNotIn(forbidden, lowered)

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
        for removed in (
                "uav_setup_topic", "uav_command_topic",
                "uav_control_state_topic", "ground_cmd_topic",
                "ground_scan_topic", "world_frame", "ground_timeout"):
            self.assertNotIn(removed, config)

    def test_sim_composition_starts_common_flight_and_navigation_layers(self):
        root = ET.parse(str(LAUNCH)).getroot()
        includes = [item.get("file") for item in root.findall("include")]
        self.assertTrue(any(
            "p450_flight_facade.launch" in item for item in includes))
        self.assertTrue(any(
            "ground_navigation.launch" in item for item in includes))
        for observer in ("air_observer.yaml", "ground_observer.yaml"):
            config = yaml.safe_load((PACKAGE / "config" / observer).read_text(
                encoding="utf-8"))
            self.assertEqual("map", config["target_frame"])


if __name__ == "__main__":
    unittest.main()
