#!/usr/bin/env python3

import importlib.util
import io
import re
import shutil
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest import mock

from bunker_sim_runtime import renderer


ROOT = Path(__file__).resolve().parents[4]
SOURCE_ROOT = ROOT / "src/vendor/bunker_description"
SOURCE = SOURCE_ROOT / "urdf/bunker.urdf.xacro"
SCRIPT = ROOT / "src/platform/bunker_sim_runtime/scripts/render_bunker_runtime.py"
NAME_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_]{0,63}$")


def _load_cli():
    spec = importlib.util.spec_from_file_location(
        "render_bunker_runtime_test_target", str(SCRIPT))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class RendererTest(unittest.TestCase):
    def _runtime(self, source=SOURCE):
        payload = renderer.render_runtime_urdf(source)
        return payload, ET.fromstring(payload)

    def _clone_source(self):
        temporary = tempfile.TemporaryDirectory(prefix="bunker-render-")
        self.addCleanup(temporary.cleanup)
        package = Path(temporary.name) / "bunker_description"
        shutil.copytree(SOURCE_ROOT, package)
        return package

    def test_runtime_has_required_robotics_interfaces(self):
        payload, root = self._runtime()
        self.assertEqual("robot", root.tag)

        links = root.findall("link")
        joints = root.findall("joint")
        for elements in (links, joints):
            names = [element.get("name") for element in elements]
            self.assertEqual(len(names), len(set(names)))
            self.assertTrue(all(NAME_RE.fullmatch(name) for name in names))

        children = {joint.find("child").get("link") for joint in joints}
        self.assertEqual(
            {"base_link"}, {link.get("name") for link in links} - children)
        self.assertTrue(all(joint.get("type") == "fixed" for joint in joints))

        collisions = root.findall(".//collision")
        self.assertEqual(1, len(collisions))
        self.assertEqual("base_link_collision", collisions[0].get("name"))
        self.assertEqual(
            renderer.EXPECTED_BOX_ORIGIN,
            collisions[0].find("origin").get("xyz"))
        self.assertEqual(
            renderer.EXPECTED_BOX_SIZE,
            collisions[0].find("geometry/box").get("size"))

        plugins = {plugin.get("name"): plugin
                   for plugin in root.findall(".//plugin")}
        self.assertEqual(
            {"bunker_laser", "bunker_planar_move"}, set(plugins))
        self.assertEqual(
            "libbunker_planar_move_plugin.so",
            plugins["bunker_planar_move"].get("filename"))
        self.assertEqual(
            "cmd_vel",
            plugins["bunker_planar_move"].findtext("commandTopic"))
        self.assertEqual(
            "bunker_status",
            plugins["bunker_planar_move"].findtext("statusTopic"))
        self.assertEqual("odom", plugins["bunker_planar_move"].findtext(
            "odometryFrame"))
        self.assertEqual(
            "libgazebo_ros_laser.so",
            plugins["bunker_laser"].get("filename"))
        self.assertEqual("scan", plugins["bunker_laser"].findtext(
            "topicName"))
        self.assertEqual("lidar_2d_link", plugins["bunker_laser"].findtext(
            "frameName"))
        self.assertEqual("720", root.findtext(".//sensor/ray/scan/horizontal/samples"))
        self.assertNotIn(str(SOURCE_ROOT), payload)

    def test_source_can_gain_a_fixed_camera_mount(self):
        package = self._clone_source()
        source = package / "urdf/bunker.urdf.xacro"
        text = source.read_text(encoding="utf-8")
        addition = """
  <link name="camera.mount" />
  <joint name="camera.mount.joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0" />
    <parent link="base_link" />
    <child link="camera.mount" />
  </joint>
"""
        source.write_text(
            text.replace("</robot>", addition + "</robot>"),
            encoding="utf-8")

        _, root = self._runtime(source)
        self.assertIsNotNone(root.find("./link[@name='camera_mount']"))
        self.assertIsNotNone(root.find("./joint[@name='camera_mount_joint']"))

    def test_rejects_missing_referenced_mesh(self):
        package = self._clone_source()
        (package / "meshes/BUNKER.STL").unlink()
        with self.assertRaises(renderer.RenderContractError):
            renderer.render_runtime_urdf(package / "urdf/bunker.urdf.xacro")

    def test_rejects_malformed_expansion_and_source_plugin(self):
        with mock.patch.object(renderer, "run_xacro", return_value=b"<robot"):
            with self.assertRaises(renderer.RenderContractError):
                renderer.render_runtime_urdf(SOURCE)

        root = ET.fromstring(renderer.run_xacro(SOURCE))
        root.append(ET.fromstring(
            '<gazebo><plugin name="unexpected" filename="bad.so"/></gazebo>'))
        with self.assertRaises(renderer.RenderContractError):
            renderer.validate_source_tree(root, SOURCE)

    def test_canonicalizes_names_and_rejects_collisions(self):
        self.assertEqual("wheel1_1_Link", renderer.canonical_local_name(
            "wheel1.1_Link"))
        root = ET.fromstring(
            '<robot name="fixture"><link name="wheel.1" />'
            '<link name="wheel-1" /></robot>')
        with self.assertRaises(renderer.RenderContractError):
            renderer.remap_urdf_names(root)

    def test_render_is_stable_and_check_urdf_valid(self):
        first = renderer.render_runtime_urdf(SOURCE)
        self.assertEqual(first, renderer.render_runtime_urdf(SOURCE))
        self.assertTrue(first.endswith("\n"))
        with tempfile.TemporaryDirectory(prefix="bunker-urdf-") as directory:
            output = Path(directory) / "bunker.urdf"
            output.write_text(first, encoding="utf-8")
            result = subprocess.run(
                ["/usr/bin/check_urdf", str(output)],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
        self.assertEqual(
            0, result.returncode,
            msg=result.stderr.decode("utf-8", "replace"))

    def test_stdout_only_cli(self):
        cli = _load_cli()
        output = io.StringIO()
        errors = io.StringIO()
        self.assertEqual(
            0, cli.main(argv=(str(SOURCE),), stdout=output, stderr=errors))
        self.assertEqual(renderer.render_runtime_urdf(SOURCE), output.getvalue())
        self.assertEqual("", errors.getvalue())
        errors = io.StringIO()
        self.assertEqual(64, cli.main(argv=(), stderr=errors))
        self.assertIn("expected one xacro path", errors.getvalue())


if __name__ == "__main__":
    unittest.main()
