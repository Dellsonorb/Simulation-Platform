#!/usr/bin/env python3

import importlib.util
import io
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src/platform/ground_manipulator_runtime"
PACKAGE_XML = PACKAGE / "package.xml"
CMAKE = PACKAGE / "CMakeLists.txt"
SETUP = PACKAGE / "setup.py"
RENDERER = PACKAGE / "src/ground_manipulator_runtime/renderer.py"
RENDER_SCRIPT = PACKAGE / "scripts/render_ground_robot.py"


def _load_renderer():
    spec = importlib.util.spec_from_file_location(
        "ground_manipulator_renderer_test_target", str(RENDERER))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_render_cli():
    source_path = str(PACKAGE / "src")
    if source_path not in sys.path:
        sys.path.insert(0, source_path)
    spec = importlib.util.spec_from_file_location(
        "render_ground_robot_cli_test_target", str(RENDER_SCRIPT))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class GroundManipulatorPlatformTest(unittest.TestCase):
    def test_package_declares_only_runtime_robot_dependencies(self):
        self.assertTrue(PACKAGE_XML.is_file(), "runtime package is missing")
        root = ET.parse(str(PACKAGE_XML)).getroot()
        self.assertEqual("ground_manipulator_runtime", root.findtext("name"))
        dependencies = {
            item.text for tag in (
                "build_depend", "build_export_depend", "exec_depend")
            for item in root.findall(tag)
        }
        self.assertTrue({
            "aubo_description",
            "bunker_description",
            "bunker_sim_runtime",
            "bunker_aubo_moveit_config",
            "controller_manager",
            "dh_ag95_description",
            "gazebo_plugins",
            "gazebo_ros",
            "gazebo_ros_control",
            "joint_state_controller",
            "position_controllers",
            "robot_state_publisher",
            "tf2_ros",
            "xacro",
        }.issubset(dependencies))
        manifest = PACKAGE_XML.read_text(encoding="utf-8").lower()
        for token in ("benchmark", "provenance", "pilot", "formal"):
            self.assertNotIn(token, manifest)

        self.assertTrue(CMAKE.is_file())
        cmake = CMAKE.read_text(encoding="utf-8")
        self.assertIn("catkin_python_setup()", cmake)
        self.assertIn("scripts/render_ground_robot.py", cmake)
        self.assertIn("DIRECTORY config launch urdf worlds", cmake)
        self.assertTrue(SETUP.is_file())

    def test_renderer_canonicalizes_and_prefixes_only_link_references(self):
        self.assertTrue(RENDERER.is_file(), "renderer is missing")
        renderer = _load_renderer()
        root = ET.fromstring("""
<robot name="bunker_aubo">
  <link name="base_link"><collision name="old"/></link>
  <link name="wheel1.1_Link"/>
  <link name="aubo_i5_base_link"/>
  <link name="shoulder_link"/>
  <link name="ag95.body"/>
  <joint name="wheel1.1_jont" type="revolute">
    <parent link="base_link"/><child link="wheel1.1_Link"/>
    <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="0" velocity="0"/>
  </joint>
  <joint name="shoulder_pan_joint" type="revolute">
    <parent link="aubo_i5_base_link"/><child link="shoulder_link"/>
    <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <transmission name="arm_trans">
    <joint name="shoulder_pan_joint"/>
  </transmission>
  <gazebo reference="base_link"><mu1>0.5</mu1></gazebo>
  <gazebo reference="shoulder_pan_joint"><implicitSpringDamper>true</implicitSpringDamper></gazebo>
  <gazebo>
    <joint name="loop.joint" type="revolute">
      <parent>ag95.body</parent><child>shoulder_link</child>
    </joint>
    <plugin name="mimic" filename="mimic.so">
      <joint>wheel1.1_jont</joint>
      <mimicJoint>shoulder_pan_joint</mimicJoint>
    </plugin>
  </gazebo>
</robot>
""")

        renderer.transform_robot_tree(root)

        self.assertEqual(
            ["ground/base_link", "ground/wheel1_1_Link",
             "ground/aubo_i5_base_link", "ground/shoulder_link",
             "ground/ag95_body"],
            [link.get("name") for link in root.findall("link")])
        wheel = root.find("./joint[@name='wheel1_1_jont']")
        self.assertEqual("fixed", wheel.get("type"))
        self.assertEqual("ground/base_link", wheel.find("parent").get("link"))
        self.assertEqual(
            "ground/wheel1_1_Link", wheel.find("child").get("link"))
        self.assertIsNone(wheel.find("axis"))
        self.assertIsNone(wheel.find("limit"))
        arm = root.find("./joint[@name='shoulder_pan_joint']")
        self.assertEqual("revolute", arm.get("type"))
        self.assertEqual(
            "shoulder_pan_joint",
            root.find("./transmission/joint").get("name"))
        self.assertEqual(
            "ground/base_link",
            root.findall("gazebo")[0].get("reference"))
        self.assertEqual(
            "shoulder_pan_joint",
            root.findall("gazebo")[1].get("reference"))
        loop = root.find("./gazebo/joint[@name='loop_joint']")
        self.assertEqual("ground/ag95_body", loop.findtext("parent"))
        self.assertEqual("ground/shoulder_link", loop.findtext("child"))
        self.assertEqual(
            "wheel1_1_jont", root.findtext("./gazebo/plugin/joint"))
        self.assertEqual(
            "shoulder_pan_joint", root.findtext("./gazebo/plugin/mimicJoint"))

        base = root.find("./link[@name='ground/base_link']")
        collisions = base.findall("collision")
        self.assertEqual(1, len(collisions))
        self.assertEqual("base_link_collision", collisions[0].get("name"))
        self.assertEqual(
            renderer.BASE_COLLISION_SIZE,
            collisions[0].find("geometry/box").get("size"))

    def test_renderer_expands_and_serializes_a_xacro_file(self):
        renderer = _load_renderer()
        source = """<?xml version="1.0"?>
<robot name="bunker_aubo" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <link name="base_link"><collision name="old"/></link>
  <link name="wheel.1"/>
  <joint name="wheel.1.joint" type="revolute">
    <parent link="base_link"/><child link="wheel.1"/>
    <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="0" velocity="0"/>
  </joint>
</robot>
"""
        with tempfile.TemporaryDirectory(prefix="ground-render-") as directory:
            path = Path(directory) / "fixture.urdf.xacro"
            path.write_text(source, encoding="utf-8")
            payload = renderer.render_ground_robot(path)
        root = ET.fromstring(payload)
        self.assertEqual("ground/base_link", root.find("link").get("name"))
        self.assertEqual("wheel_1_joint", root.find("joint").get("name"))
        self.assertTrue(payload.startswith("<?xml"))
        self.assertTrue(payload.endswith("\n"))

    def test_renderer_cli_is_quiet_and_reports_input_errors(self):
        self.assertTrue(RENDER_SCRIPT.is_file(), "renderer CLI is missing")
        self.assertTrue(RENDER_SCRIPT.stat().st_mode & 0o111)
        cli = _load_render_cli()
        output = io.StringIO()
        errors = io.StringIO()
        self.assertEqual(64, cli.main(argv=(), stdout=output, stderr=errors))
        self.assertEqual("", output.getvalue())
        self.assertIn("expected one xacro path", errors.getvalue())

        errors = io.StringIO()
        self.assertEqual(
            65,
            cli.main(
                argv=("/definitely/missing/ground.urdf.xacro",),
                stdout=output, stderr=errors))
        self.assertIn("xacro file is missing", errors.getvalue())


if __name__ == "__main__":
    unittest.main()
