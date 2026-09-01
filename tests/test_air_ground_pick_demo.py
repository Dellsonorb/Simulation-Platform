#!/usr/bin/env python3

import importlib.util
import math
from pathlib import Path
import unittest
import warnings
import xml.etree.ElementTree as ET

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src/demos/air_ground_pick_demo"
PACKAGE_XML = PACKAGE / "package.xml"
CMAKE = PACKAGE / "CMakeLists.txt"
PERCEPTION = (
    PACKAGE / "src/air_ground_pick_demo/perception.py")
TARGET_MODEL = PACKAGE / "models/pick_target/model.sdf"
TARGET_CONFIG = PACKAGE / "models/pick_target/model.config"
OBSERVER = PACKAGE / "scripts/red_target_observer.py"
AIR_CONFIG = PACKAGE / "config/air_observer.yaml"
GROUND_CONFIG = PACKAGE / "config/ground_observer.yaml"
OBSERVERS_LAUNCH = PACKAGE / "launch/target_observers.launch"


def _load_perception():
    if not PERCEPTION.is_file():
        raise AssertionError("minimal pick perception module is missing")
    spec = importlib.util.spec_from_file_location(
        "air_ground_pick_perception_test_target", str(PERCEPTION))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class MinimalAirGroundPickDemoTest(unittest.TestCase):
    def test_package_is_a_small_sim_demo_layer(self):
        root = ET.parse(str(PACKAGE_XML)).getroot()
        self.assertEqual("air_ground_pick_demo", root.findtext("name"))
        dependencies = {
            item.text for tag in ("build_depend", "exec_depend")
            for item in root.findall(tag)}
        for required in (
                "cv_bridge", "geometry_msgs", "message_filters", "rospy",
                "sensor_msgs", "tf2_ros"):
            self.assertIn(required, dependencies)
        cmake = CMAKE.read_text(encoding="utf-8")
        self.assertIn("catkin_python_setup", cmake)
        self.assertIn("DIRECTORY models", cmake)
        combined = cmake.lower() + PACKAGE_XML.read_text(
            encoding="utf-8").lower()
        for forbidden in (
                "benchmark", "provenance", "artifact", "pilot", "formal"):
            self.assertNotIn(forbidden, combined)

    def test_pick_target_is_dynamic_contact_sensed_and_gripper_feasible(self):
        model = ET.parse(str(TARGET_MODEL)).getroot().find("model")
        self.assertIsNotNone(model)
        self.assertEqual("pick_target", model.get("name"))
        self.assertNotEqual("true", model.findtext("static", "false"))
        link = model.find("link")
        self.assertEqual("pick_target_link", link.get("name"))
        self.assertAlmostEqual(0.20, float(link.findtext("inertial/mass")))
        inertia = link.find("inertial/inertia")
        for name in ("ixx", "iyy", "izz"):
            self.assertGreater(float(inertia.findtext(name)), 0.0)
        collision = link.find("collision")
        size = tuple(float(value) for value in
                     collision.findtext("geometry/box/size").split())
        self.assertEqual((0.240, 0.053, 0.115), size)
        self.assertLess(size[1] + 0.002, 0.0952)
        surface = collision.find("surface")
        self.assertGreaterEqual(float(surface.findtext("friction/ode/mu")), 1.0)
        self.assertGreaterEqual(float(surface.findtext("friction/ode/mu2")), 1.0)
        material = link.find("visual/material")
        self.assertEqual(
            "Gazebo/Red", material.findtext("script/name"))
        self.assertEqual(
            "file://media/materials/scripts/gazebo.material",
            material.findtext("script/uri"))
        sensor = link.find("sensor[@type='contact']")
        self.assertIsNotNone(sensor)
        self.assertEqual(
            collision.get("name"), sensor.findtext("contact/collision"))
        plugin = sensor.find("plugin")
        self.assertEqual("libgazebo_ros_bumper.so", plugin.get("filename"))
        self.assertEqual("/pick_target/contacts",
                         plugin.findtext("bumperTopicName"))
        self.assertTrue(TARGET_CONFIG.is_file())

    def test_red_component_selection_rejects_ambiguity(self):
        perception = _load_perception()
        image = np.zeros((100, 120, 3), dtype=np.uint8)
        image[30:70, 40:90, 0] = 230
        mask = perception.select_red_component(
            image, min_pixels=500, ambiguity_ratio=0.80)
        self.assertEqual(image.shape[:2], mask.shape)
        self.assertGreaterEqual(int(np.count_nonzero(mask)), 1900)
        self.assertEqual(0, int(np.count_nonzero(mask[:, :30])))

        image[10:30, 5:35, 0] = 230
        with self.assertRaises(perception.PerceptionError):
            perception.select_red_component(
                image, min_pixels=500, ambiguity_ratio=0.30)

    def test_red_component_selection_rejects_aubo_orange(self):
        perception = _load_perception()
        image = np.zeros((60, 80, 3), dtype=np.uint8)
        image[10:50, 20:60] = (200, 70, 0)
        with self.assertRaises(perception.PerceptionError):
            perception.select_red_component(image, min_pixels=100)

    def test_backprojection_uses_calibration_and_rejects_invalid_depth(self):
        perception = _load_perception()
        depth = np.full((3, 3), np.nan, dtype=np.float32)
        depth[1, 2] = 2.0
        mask = np.zeros((3, 3), dtype=np.uint8)
        mask[1, 2] = 255
        points = perception.backproject_mask(
            mask, depth, (2.0, 0.0, 1.0,
                          0.0, 2.0, 1.0,
                          0.0, 0.0, 1.0),
            min_depth=0.2, max_depth=4.0)
        np.testing.assert_allclose(points, [[1.0, 0.0, 2.0]])
        depth[1, 2] = 0.0
        with self.assertRaises(perception.PerceptionError):
            perception.backproject_mask(
                mask, depth, np.eye(3), min_depth=0.2, max_depth=4.0)

    def test_pose_estimation_fits_top_surface_and_pi_periodic_yaw(self):
        perception = _load_perception()
        yaw = 0.38
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        samples = []
        for along in np.linspace(-0.12, 0.12, 25):
            for across in np.linspace(-0.0265, 0.0265, 9):
                samples.append((
                    1.7 + cosine * along - sine * across,
                    -0.2 + sine * along + cosine * across,
                    0.115))
        pose = perception.estimate_target_pose(
            np.asarray(samples), target_height=0.115,
            top_surface_tolerance=0.01)
        self.assertAlmostEqual(1.7, pose[0], places=3)
        self.assertAlmostEqual(-0.2, pose[1], places=3)
        self.assertAlmostEqual(0.0575, pose[2], places=3)
        self.assertLess(perception.yaw_error_mod_pi(pose[3], yaw), 0.01)

    def test_pose_fusion_requires_consistent_finite_sensor_samples(self):
        perception = _load_perception()
        samples = (
            (2.00, 0.01, 0.057, 0.02),
            (2.01, 0.00, 0.058, 0.01),
            (1.99, -0.01, 0.0575, math.pi - 0.01),
        )
        fused = perception.fuse_pose_samples(
            samples, max_position_spread=0.04, max_yaw_spread=0.08)
        np.testing.assert_allclose(fused[:3], [2.0, 0.0, 0.0575], atol=1e-3)
        self.assertLess(perception.yaw_error_mod_pi(fused[3], 0.0), 0.02)
        inconsistent = samples + ((2.30, 0.0, 0.0575, 0.0),)
        with self.assertRaises(perception.PerceptionError):
            perception.fuse_pose_samples(
                inconsistent, max_position_spread=0.04,
                max_yaw_spread=0.08)

    def test_depth_registration_uses_real_optical_extrinsics(self):
        perception = _load_perception()
        depth = np.full((3, 3), np.nan, dtype=np.float32)
        depth[1, 1] = 2.0
        with warnings.catch_warnings():
            warnings.simplefilter("error", RuntimeWarning)
            registered = perception.register_depth_to_color(
                depth, np.eye(3), np.eye(3), (3, 3),
                rotation=np.eye(3), translation=np.array((1.0, 0.0, 0.0)))
        self.assertTrue(math.isnan(float(registered[1, 1])))
        self.assertAlmostEqual(2.0, float(registered[1, 2]))
        with self.assertRaises(perception.PerceptionError):
            perception.register_depth_to_color(
                depth, np.eye(3), np.eye(3), (3, 3),
                rotation=np.zeros((2, 2)), translation=np.zeros(3))

    def test_observation_timestamps_must_be_current_sim_time(self):
        perception = _load_perception()
        perception.validate_observation_stamps(
            (9.96, 9.98, 9.96, 9.98), now=10.0,
            max_age=0.5, max_future_skew=0.1)
        with self.assertRaises(perception.PerceptionError):
            perception.validate_observation_stamps(
                (8.0, 8.0, 8.0, 8.0), now=10.0,
                max_age=0.5, max_future_skew=0.1)
        with self.assertRaises(perception.PerceptionError):
            perception.validate_observation_stamps(
                (10.2, 10.2, 10.2, 10.2), now=10.0,
                max_age=0.5, max_future_skew=0.1)

    def test_observers_are_camera_configured_and_have_no_gt_input(self):
        source = OBSERVER.read_text(encoding="utf-8")
        for required in (
                "message_filters.ApproximateTimeSynchronizer",
                "select_red_component", "register_depth_to_color",
                "backproject_mask", "estimate_target_pose",
                "fuse_pose_samples", "lookup_transform",
                "validate_observation_stamps",
                "PoseStamped", "rospy.is_shutdown()",
                "def publish_pose",
                "color_frame, depth_frame, stamps =",
                "color.header.frame_id", "color_info.header.frame_id",
                "depth_info.header.frame_id", "to_sec()"):
            self.assertIn(required, source)
        for parameter in (
                "~color_topic", "~depth_topic", "~color_info_topic",
                "~depth_info_topic", "~output_topic", "~target_frame",
                "~camera_optical_frame"):
            self.assertIn(parameter, source)
        lowered = source.lower()
        for forbidden in (
                "/gazebo/model", "getmodelstate", "setmodelstate",
                "teleport", "attach", "benchmark", "provenance"):
            self.assertNotIn(forbidden, lowered)
        self.assertGreaterEqual(source.count("validate_observation_stamps("), 2)

    def test_air_and_ground_observer_configs_use_public_d435_topics(self):
        import yaml

        air = yaml.safe_load(AIR_CONFIG.read_text(encoding="utf-8"))
        ground = yaml.safe_load(GROUND_CONFIG.read_text(encoding="utf-8"))
        self.assertEqual(
            "/uav1/camera/color/image_raw", air["color_topic"])
        self.assertEqual(
            "/uav1/camera/depth/image_raw", air["depth_topic"])
        self.assertEqual(
            "uav1/camera_link",
            air["camera_optical_frame"])
        self.assertEqual("/air_observer/target_pose", air["output_topic"])
        self.assertEqual(
            "/ground/d435/color/image_raw", ground["color_topic"])
        self.assertEqual(
            "/ground/d435/depth/image_raw", ground["depth_topic"])
        self.assertEqual(
            "ground/d435_color_optical_frame",
            ground["camera_optical_frame"])
        self.assertEqual(
            "/ground_observer/target_pose", ground["output_topic"])
        for config in (air, ground):
            self.assertEqual("world", config["target_frame"])
            self.assertEqual(0.115, config["target_height"])
            self.assertGreaterEqual(config["stable_frames"], 3)
            self.assertEqual(1.0, config["max_observation_age"])
            self.assertGreaterEqual(config["max_future_skew"], 0.0)

    def test_observer_launch_starts_two_independent_sensor_nodes(self):
        root = ET.parse(str(OBSERVERS_LAUNCH)).getroot()
        nodes = root.findall("node")
        self.assertEqual(
            {"air_target_observer", "ground_target_observer"},
            {node.get("name") for node in nodes})
        for node in nodes:
            self.assertEqual("air_ground_pick_demo", node.get("pkg"))
            self.assertEqual("red_target_observer.py", node.get("type"))
            rosparam = node.find("rosparam")
            self.assertIsNotNone(rosparam)
            self.assertIn("_observer.yaml", rosparam.get("file"))


if __name__ == "__main__":
    unittest.main()
