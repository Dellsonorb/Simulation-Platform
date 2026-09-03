#!/usr/bin/env python3

import math
import json
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "src"))

from rm4d_sim_integration.geometry import CandidateValues  # noqa: E402
from rm4d_sim_integration.markers import marker_specs  # noqa: E402


class RosContractTest(unittest.TestCase):
    def test_service_definition_is_minimal_and_ordered(self):
        lines = [
            line.strip()
            for line in (PACKAGE / "srv/PlanBasePlacement.srv").read_text().splitlines()
            if line.strip() and not line.lstrip().startswith("#")]
        self.assertEqual([
            "geometry_msgs/PoseStamped grasp_tcp",
            "string grasp_id",
            "uint32 top_k",
            "---",
            "bool success",
            "string status",
            "geometry_msgs/PoseArray candidates",
            "string[] candidate_ids",
            "float64[] scores",
            "string message",
        ], lines)

    def test_catkin_declares_only_adapter_dependencies(self):
        root = ET.parse(PACKAGE / "package.xml").getroot()
        dependencies = {
            element.text for element in root
            if element.tag.endswith("depend") and element.text}
        self.assertTrue({
            "geometry_msgs", "message_generation", "message_runtime",
            "rospy", "std_msgs", "tf2_ros", "visualization_msgs",
        }.issubset(dependencies))
        self.assertNotIn("gazebo_msgs", dependencies)
        self.assertNotIn("move_base_msgs", dependencies)

    def test_launch_requires_external_paths_and_interpreter(self):
        root = ET.parse(PACKAGE / "launch/rm4d_adapter.launch").getroot()
        arguments = {item.get("name"): item for item in root.findall("arg")}
        self.assertIn("rm4d_root", arguments)
        self.assertIn("rm4d_python", arguments)
        self.assertIn("rm4d_map", arguments)
        self.assertNotIn("''", " ".join(
            item.get("default", "") for item in arguments.values()))
        node = root.find("node[@type='rm4d_adapter_node.py']")
        self.assertIsNotNone(node)
        self.assertEqual("$(arg rm4d_python)", node.get("launch-prefix"))
        params = {item.get("name"): item.get("value")
                  for item in node.findall("param")}
        self.assertEqual("map", params["map_frame"])
        self.assertEqual("ground/base_link", params["ground_base_frame"])
        process_environment = {
            item.get("name"): item.get("value")
            for item in node.findall("env")}
        self.assertIn(
            "$(find rm4d_sim_integration)/python_compat",
                      process_environment["PYTHONPATH"])
        self.assertIn("$(optenv PYTHONPATH)",
                      process_environment["PYTHONPATH"])
        compatibility = (
            PACKAGE / "python_compat/sitecustomize.py").read_text()
        self.assertIn("sys.path.append", compatibility)
        self.assertIn("/usr/lib/python3/dist-packages", compatibility)

    def test_node_is_map_tf_api_and_visualization_only(self):
        source = (PACKAGE / "scripts/rm4d_adapter_node.py").read_text()
        for required in (
                "load_external_api", "lookup_transform",
                'Publisher("/rm4d/grasp_tcp"',
                'Publisher("/rm4d/candidates"',
                'Publisher("/rm4d/candidate_markers"',
                "latch=True", "PlanBasePlacement"):
            self.assertIn(required, source)
        for forbidden in (
                "gazebo_msgs", "ModelStates", "/gazebo/", "SetModelState",
                "move_base_msgs", "cmd_vel"):
            self.assertNotIn(forbidden, source)
        self.assertEqual(1, source.count("load_external_api("))
        self.assertIn("self._api.close", source)


class MarkerSpecificationTest(unittest.TestCase):
    def test_top_one_is_distinct_and_footprints_keep_rank(self):
        candidates = (
            CandidateValues("candidate-000162", 2.5, 0.3, 0.0, 0.95),
            CandidateValues("candidate-000013", 2.0, -0.2,
                            math.pi / 2.0, 0.90),
        )

        specs = marker_specs(candidates)

        self.assertEqual(6, len(specs))
        arrows = [item for item in specs if item.kind == "arrow"]
        footprints = [item for item in specs if item.kind == "footprint"]
        labels = [item for item in specs if item.kind == "text"]
        self.assertEqual([0, 1], [item.rank for item in arrows])
        self.assertNotEqual(arrows[0].color, arrows[1].color)
        self.assertEqual("#1 candidate-000162 score=0.950000",
                         labels[0].text)
        first_x = [point[0] for point in footprints[0].points[:-1]]
        first_y = [point[1] for point in footprints[0].points[:-1]]
        self.assertAlmostEqual(1.04, max(first_x) - min(first_x))
        self.assertAlmostEqual(0.78, max(first_y) - min(first_y))
        self.assertEqual(footprints[0].points[0],
                         footprints[0].points[-1])


class SensorReplayContractTest(unittest.TestCase):
    def test_replay_source_is_sensor_observation_in_map(self):
        replay = yaml.safe_load(
            (PACKAGE / "config/p450_rgbd_replay.yaml").read_text())
        self.assertEqual("/air_observer/target_pose", replay["source_topic"])
        self.assertEqual("map", replay["frame_id"])
        self.assertEqual([
            1.9899721371390122,
            0.004464723547961525,
            0.023386687889514802,
            0.008026569019304208,
        ], replay["target"])
        self.assertNotIn("gazebo", json.dumps(replay).lower())

    def test_replay_uses_exact_existing_grasp_and_both_api_paths(self):
        source = (
            PACKAGE / "scripts/replay_rm4d_observation.py").read_text()
        self.assertIn("generate_top_down_grasp", source)
        self.assertIn("IntegrationCore", source)
        self.assertIn("PlanBasePlacement", source)
        self.assertIn('choices=("offline", "online")', source)
        self.assertNotIn("regularize_for_rm4d", source)


if __name__ == "__main__":
    unittest.main()
