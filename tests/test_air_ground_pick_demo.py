#!/usr/bin/env python3

import importlib.util
import math
from pathlib import Path
import unittest
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


if __name__ == "__main__":
    unittest.main()
