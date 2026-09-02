#!/usr/bin/env python3

import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE / "launch/ground_navigation.launch"
STOP_SCRIPT = PACKAGE / "scripts/ground_stop_server.py"
SOURCE_ROOT = PACKAGE / "src"


class CommonNavigationTest(unittest.TestCase):
    def test_launch_exposes_one_namespaced_move_base_without_sim_backend(self):
        root = ET.parse(str(LAUNCH)).getroot()
        groups = root.findall("./group[@ns='ground']")
        self.assertEqual(1, len(groups))
        move_base = groups[0].findall("./node[@type='move_base']")
        self.assertEqual(1, len(move_base))
        self.assertEqual("move_base", move_base[0].get("name"))
        self.assertEqual(
            ["nav_cmd_vel"],
            [item.get("to") for item in
             move_base[0].findall("./remap[@from='cmd_vel']")])

        text = LAUNCH.read_text(encoding="utf-8").lower()
        for forbidden in (
                "gazebo", "heading_gate", "lifecycle", "world_name",
                "bunker_sim_runtime", "bunker_base"):
            self.assertNotIn(forbidden, text)
        stop = groups[0].find("./node[@type='ground_stop_server.py']")
        self.assertIsNotNone(stop)
        self.assertEqual("true", stop.get("required"))

    def test_costmaps_use_shared_map_local_odom_and_standard_topics(self):
        common = yaml.safe_load((
            PACKAGE / "config/costmap_common.yaml").read_text(
                encoding="utf-8"))
        global_map = yaml.safe_load((
            PACKAGE / "config/global_costmap.yaml").read_text(
                encoding="utf-8"))["global_costmap"]
        local_map = yaml.safe_load((
            PACKAGE / "config/local_costmap.yaml").read_text(
                encoding="utf-8"))["local_costmap"]
        planner = yaml.safe_load((
            PACKAGE / "config/local_planner.yaml").read_text(
                encoding="utf-8"))["DWAPlannerROS"]

        self.assertEqual("ground/base_link", common["robot_base_frame"])
        self.assertNotIn("global_frame", common)
        self.assertEqual("/ground/scan", common["obstacle_layer"]["scan"]["topic"])
        self.assertEqual(
            "ground/lidar_2d_link",
            common["obstacle_layer"]["scan"]["sensor_frame"])
        self.assertEqual("map", global_map["global_frame"])
        self.assertEqual("ground/odom", local_map["global_frame"])
        self.assertEqual("/ground/odom", planner["odom_topic"])
        self.assertFalse(planner["holonomic_robot"])

    def test_local_planner_can_finish_a_short_reverse_approach_and_turn(self):
        planner = yaml.safe_load((
            PACKAGE / "config/local_planner.yaml").read_text(
                encoding="utf-8"))["DWAPlannerROS"]

        # The manipulation standoff is a short reverse move followed by a
        # 180-degree final heading.  A forward scoring point makes DWA orbit
        # that nearby goal instead of entering its rotate-to-goal behavior.
        self.assertEqual(0.0, planner["forward_point_distance"])
        self.assertEqual(0.0, planner["min_vel_trans"])
        self.assertEqual(0.025, planner["xy_goal_tolerance"])

    def test_stop_helper_cancels_navigation_and_publishes_zero(self):
        sys.path.insert(0, str(SOURCE_ROOT))
        try:
            from bunker_navigation.stop import stop_robot
        finally:
            sys.path.pop(0)

        class Client:
            cancellations = 0

            def cancel_all_goals(self):
                self.cancellations += 1

        class Publisher:
            messages = []

            def publish(self, message):
                self.messages.append(message)

        def twist():
            return SimpleNamespace(
                linear=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                angular=SimpleNamespace(x=0.0, y=0.0, z=0.0))

        client = Client()
        publisher = Publisher()
        result = stop_robot(client, publisher, twist)
        self.assertEqual(1, client.cancellations)
        self.assertEqual(1, len(publisher.messages))
        self.assertEqual(0.0, publisher.messages[0].linear.x)
        self.assertEqual(0.0, publisher.messages[0].angular.z)
        self.assertEqual({"success": True, "message": "ground stopped"}, result)

    def test_stop_server_owns_only_the_common_navigation_boundary(self):
        source = STOP_SCRIPT.read_text(encoding="utf-8")
        self.assertIn('SimpleActionClient("move_base", MoveBaseAction)', source)
        self.assertIn('Publisher("nav_cmd_vel", Twist', source)
        self.assertIn('"runtime/stop", Trigger', source)
        for forbidden in (
                "gazebo", "cmd_vel_safe", "/ground/cmd_vel", "world"):
            self.assertNotIn(forbidden, source.lower())

    def test_package_build_has_no_gazebo_or_hardware_adapter_dependency(self):
        package = ET.parse(str(PACKAGE / "package.xml")).getroot()
        dependencies = {
            item.text for label in (
                "build_depend", "build_export_depend", "exec_depend")
            for item in package.findall(label)}
        self.assertIn("move_base", dependencies)
        self.assertIn("std_srvs", dependencies)
        self.assertFalse({
            "gazebo_dev", "gazebo_msgs", "gazebo_ros",
            "bunker_aubo_gazebo", "bunker_sim_runtime",
        }.intersection(dependencies))
        cmake = (PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("scripts/ground_stop_server.py", cmake)
        self.assertNotIn("bunker_planar_drive_plugin", cmake)
        self.assertNotIn("find_package(gazebo", cmake.lower())


if __name__ == "__main__":
    unittest.main()
