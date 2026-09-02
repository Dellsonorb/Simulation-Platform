#!/usr/bin/env python3

import re
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
VENDOR = ROOT / "src/vendor/bunker_description"
RUNTIME = ROOT / "src/platform/bunker_sim_runtime"


class BunkerModelPackageTest(unittest.TestCase):
    def test_model_references_only_present_package_meshes(self):
        xacro = VENDOR / "urdf/bunker.urdf.xacro"
        root = ET.parse(str(xacro)).getroot()
        meshes = root.findall(".//mesh")
        self.assertGreater(len(meshes), 0)
        prefix = "package://bunker_description/"
        for mesh in meshes:
            uri = mesh.get("filename")
            self.assertTrue(uri.startswith(prefix), uri)
            relative = uri[len(prefix):]
            self.assertNotIn("..", Path(relative).parts)
            self.assertTrue((VENDOR / relative).is_file(), relative)

    def test_vendor_package_installs_only_model_resources(self):
        cmake = (VENDOR / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertRegex(
            cmake,
            r"install\s*\(\s*DIRECTORY\s+meshes\s+urdf\s+"
            r"DESTINATION\s+\$\{CATKIN_PACKAGE_SHARE_DESTINATION\}\s*\)",
        )
        self.assertNotRegex(cmake, r"DIRECTORY[^\)]*(launch|rviz|config)")

    def test_bunker_has_no_runtime_asset_lock_pipeline(self):
        self.assertFalse((ROOT / "config/bunker_assets.json").exists())
        self.assertFalse((ROOT / "tools/bunker_assets.py").exists())


class BunkerRuntimePackageTest(unittest.TestCase):
    def test_package_declares_robot_runtime_dependencies(self):
        package = ET.parse(str(RUNTIME / "package.xml")).getroot()
        runtime_dependencies = {
            item.text for tag in ("depend", "exec_depend")
            for item in package.findall(tag)}
        self.assertTrue({
            "bunker_description", "bunker_msgs", "gazebo_plugins", "gazebo_ros",
            "geometry_msgs", "nav_msgs", "robot_state_publisher", "rospy",
            "sensor_msgs", "tf2_ros", "xacro",
        }.issubset(runtime_dependencies))

    def test_cmake_installs_only_runtime_programs_launch_and_worlds(self):
        cmake = (RUNTIME / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("scripts/velocity_guard.py", cmake)
        self.assertIn("scripts/render_bunker_runtime.py", cmake)
        self.assertRegex(cmake, r"DIRECTORY\s+launch\s+worlds")
        self.assertIn("add_library(bunker_planar_move_plugin", cmake)
        self.assertIn("TARGETS bunker_planar_move_plugin", cmake)
        self.assertNotIn("spawn_bunker_preflight", cmake)
        self.assertNotIn("test/test_contracts.py", cmake)
        self.assertNotIn("test/test_sim_time.py", cmake)

    def test_preflight_barrier_modules_are_absent(self):
        self.assertFalse((RUNTIME / "scripts/spawn_bunker_preflight.py").exists())
        self.assertFalse(
            (RUNTIME / "src/bunker_sim_runtime/contracts.py").exists())
        self.assertFalse(
            (RUNTIME / "src/bunker_sim_runtime/sim_time.py").exists())

    def test_profile_is_an_installed_noetic_workspace(self):
        profile = yaml.safe_load((
            ROOT / ".catkin_tools/profiles/p450-clean/config.yaml"
        ).read_text(encoding="utf-8"))
        self.assertTrue(profile["install"])
        self.assertEqual("install/p450-clean", profile["install_space"])
        self.assertEqual("/opt/ros/noetic", profile["extend_path"])
        combined = "\n".join(str(value) for value in profile.values())
        self.assertNotIn("P450-PAPER", combined)

    def test_local_planar_plugin_is_robot_only_and_has_ordered_shutdown(self):
        source_path = RUNTIME / "src/bunker_planar_move_plugin.cpp"
        self.assertTrue(source_path.is_file())
        source = source_path.read_text(encoding="utf-8")
        self.assertIn("~BunkerPlanarMovePlugin", source)
        self.assertIn("callback_thread_.join()", source)
        self.assertIn("GetLink(StripLeadingSlash(robot_base_frame_))", source)
        self.assertIn("base_link_->SetLinearVel", source)
        self.assertIn("base_link_->SetAngularVel", source)
        self.assertIn("bunker_msgs::BunkerStatus", source)
        self.assertIn("status_publisher_", source)
        self.assertIn('"commandTopic", "cmd_vel"', source)
        self.assertIn('"statusTopic", "bunker_status"', source)
        self.assertIn("odometry_x_", source)
        self.assertIn("odometry_y_", source)
        self.assertIn("odometry_yaw_", source)
        publish_body = source.split("void PublishOdometry", 1)[1]
        self.assertNotIn("WorldPose", publish_body)
        self.assertNotIn("pose.Pos()", publish_body)
        self.assertNotIn("pose.Rot()", publish_body)
        self.assertIn("bunker_msgs", (
            RUNTIME / "CMakeLists.txt").read_text(encoding="utf-8"))
        self.assertNotIn("model_->SetLinearVel", source)
        self.assertNotIn("model_->SetAngularVel", source)
        self.assertNotIn("model_->SetWorldTwist", source)
        for forbidden in (
                "SetWorldPose", "SetKinematic", "CreateJoint", "attachment",
                "handoff", "AUBO", "AG95", "benchmark"):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, source)


if __name__ == "__main__":
    unittest.main()
