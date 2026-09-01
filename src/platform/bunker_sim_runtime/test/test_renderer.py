import hashlib
import importlib.util
import io
import os
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
SCRIPT = ROOT / (
    "src/platform/bunker_sim_runtime/scripts/render_bunker_runtime.py")
TEMP_ROOT = ROOT / "logs/bunker_standalone/engineering-tmp"
EXPECTED_COUNTS = {
    "link": 18,
    "joint": 17,
    "collision": 1,
    "sensor": 1,
    "plugin": 2,
}
EXPECTED_BOX_ORIGIN = "0.018058912 0.001357451 -0.160420741"
EXPECTED_BOX_SIZE = "1.026335219 0.782744936 0.395154782"
EXPECTED_RUNTIME_URDF_SHA256 = (
    "2b58856bed616a9e7402f7ddf4be4336f669271ca3989fdcf1d50f6939270832")
EXPECTED_PLUGIN_FILES = {
    "libgazebo_ros_planar_move.so",
    "libgazebo_ros_laser.so",
}
INCOMPATIBLE_JOINT_TAGS = (
    "axis", "limit", "dynamics", "calibration", "mimic",
    "safety_controller",
)

LIDAR_GAZEBO_XML = """<gazebo reference="lidar_2d_link">
  <sensor name="bunker_lidar_2d" type="ray">
    <always_on>true</always_on>
    <update_rate>15.0</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-3.141592653589793</min_angle>
          <max_angle>3.141592653589793</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.12</min>
        <max>8.0</max>
        <resolution>0.01</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0</mean>
        <stddev>0.002</stddev>
      </noise>
    </ray>
    <plugin name="bunker_laser" filename="libgazebo_ros_laser.so">
      <robotNamespace>/ground</robotNamespace>
      <topicName>scan</topicName>
      <frameName>lidar_2d_link</frameName>
    </plugin>
  </sensor>
</gazebo>"""
LIDAR_JOINT_XML = """<joint name="lidar_2d_joint" type="fixed">
  <origin xyz="-0.30 0.0 0.25" rpy="0 0 0" />
  <parent link="base_link" />
  <child link="lidar_2d_link" />
</joint>"""
PLANAR_GAZEBO_XML = """<gazebo>
  <plugin name="bunker_planar_move" filename="libgazebo_ros_planar_move.so">
    <robotNamespace>/ground</robotNamespace>
    <commandTopic>cmd_vel_safe</commandTopic>
    <odometryTopic>odom</odometryTopic>
    <odometryFrame>odom</odometryFrame>
    <robotBaseFrame>base_link</robotBaseFrame>
    <odometryRate>50.0</odometryRate>
    <cmdTimeout>0.5</cmdTimeout>
  </plugin>
</gazebo>"""


def _signature(element):
    return (
        element.tag,
        tuple(sorted(element.attrib.items())),
        (element.text or "").strip(),
        tuple(_signature(child) for child in list(element)),
    )


def _load_cli():
    spec = importlib.util.spec_from_file_location(
        "render_bunker_runtime_test_target", str(SCRIPT))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class RendererTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        TEMP_ROOT.mkdir(mode=0o700, parents=True, exist_ok=True)
        if TEMP_ROOT.is_symlink() or (TEMP_ROOT.stat().st_mode & 0o777) != 0o700:
            raise RuntimeError("renderer temp root must be mode 0700")

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(dir=str(TEMP_ROOT))
        self.addCleanup(self.temporary.cleanup)

    def _clone_source(self):
        destination = Path(self.temporary.name) / "bunker_description"
        shutil.copytree(SOURCE_ROOT, destination, copy_function=os.link)
        return destination

    def _expanded_source(self):
        return ET.fromstring(renderer.run_xacro(SOURCE))

    def _runtime(self):
        payload = renderer.render_runtime_urdf(SOURCE)
        return payload, ET.fromstring(payload)

    def test_canonicalizes_names_without_correcting_source_typos(self):
        expected = {
            "base_link": "base_link",
            "wheel1.1_Link": "wheel1_1_Link",
            "wheel1.1_jont": "wheel1_1_jont",
            "wheel4.3_joint": "wheel4_3_joint",
        }
        for source, target in expected.items():
            with self.subTest(source=source):
                self.assertEqual(target, renderer.canonical_local_name(source))
        self.assertEqual(
            "wheel_1_Link",
            renderer.canonical_local_name("wheel..--1_Link"),
        )

    def test_rejects_name_collisions_after_canonicalization(self):
        root = ET.fromstring(
            '<robot name="fixture"><link name="wheel.1" />'
            '<link name="wheel-1" /></robot>')
        with self.assertRaises(renderer.RenderContractError):
            renderer.remap_urdf_names(root)

    def test_remaps_declarations_and_all_supported_references(self):
        root = ET.fromstring(
            '<robot name="fixture">'
            '<link name="base.link"/><link name="wheel.1"/>'
            '<joint name="joint.1" type="fixed">'
            '<parent link="base.link"/><child link="wheel.1"/>'
            '</joint>'
            '<joint name="joint.2" type="fixed">'
            '<parent link="base.link"/><child link="wheel.1"/>'
            '<mimic joint="joint.1"/></joint>'
            '<gazebo reference="wheel.1"/>'
            '</robot>')
        renderer.remap_urdf_names(root)
        self.assertEqual(
            ["base_link", "wheel_1"],
            [item.get("name") for item in root.findall("link")],
        )
        self.assertEqual(
            ["joint_1", "joint_2"],
            [item.get("name") for item in root.findall("joint")],
        )
        self.assertEqual(
            "base_link", root.find("joint/parent").get("link"))
        self.assertEqual(
            "wheel_1", root.find("joint/child").get("link"))
        self.assertEqual(
            "joint_1", root.findall("joint")[1].find("mimic").get("joint"))
        self.assertEqual("wheel_1", root.find("gazebo").get("reference"))

    def test_rejects_wrong_input_path_hash_and_symlinks(self):
        with self.assertRaises(renderer.RenderContractError):
            renderer.render_runtime_urdf(
                SOURCE_ROOT / "urdf/bunker.urdf")

        package = self._clone_source()
        xacro = package / "urdf/bunker.urdf.xacro"
        xacro.unlink()
        xacro.write_bytes(SOURCE.read_bytes() + b"\n")
        with self.assertRaises(renderer.RenderContractError):
            renderer.render_runtime_urdf(xacro)

        shutil.rmtree(package)
        package = self._clone_source()
        xacro = package / "urdf/bunker.urdf.xacro"
        xacro.unlink()
        xacro.symlink_to(SOURCE)
        with self.assertRaises(renderer.RenderContractError):
            renderer.render_runtime_urdf(xacro)

    def test_rejects_symlinked_missing_and_nonregular_meshes(self):
        package = self._clone_source()
        mesh = package / "meshes/BUNKER.STL"
        mesh.unlink()
        mesh.symlink_to(SOURCE_ROOT / "meshes/BUNKER.STL")
        with self.assertRaises(renderer.RenderContractError):
            renderer.render_runtime_urdf(package / "urdf/bunker.urdf.xacro")

        shutil.rmtree(package)
        package = self._clone_source()
        mesh = package / "meshes/BUNKER.STL"
        mesh.unlink()
        with self.assertRaises(renderer.RenderContractError):
            renderer.render_runtime_urdf(package / "urdf/bunker.urdf.xacro")

        shutil.rmtree(package)
        package = self._clone_source()
        mesh = package / "meshes/BUNKER.STL"
        mesh.unlink()
        os.mkfifo(str(mesh))
        with self.assertRaises(renderer.RenderContractError):
            renderer.render_runtime_urdf(package / "urdf/bunker.urdf.xacro")

    def test_rejects_malformed_expansion_changed_counts_and_source_plugin(self):
        with mock.patch.object(renderer, "run_xacro", return_value=b"<robot"):
            with self.assertRaises(renderer.RenderContractError):
                renderer.render_runtime_urdf(SOURCE)

        root = self._expanded_source()
        root.remove(root.find("link"))
        with self.assertRaises(renderer.RenderContractError):
            renderer.validate_source_tree(root, SOURCE)

        root = self._expanded_source()
        root.append(ET.fromstring(
            '<gazebo><plugin name="bad" filename="bad.so"/></gazebo>'))
        with self.assertRaises(renderer.RenderContractError):
            renderer.validate_source_tree(root, SOURCE)

    def test_runtime_entity_counts_names_and_tree_root_are_exact(self):
        payload, root = self._runtime()
        for tag, count in EXPECTED_COUNTS.items():
            self.assertEqual(count, len(root.findall(".//" + tag)))
        for tag in EXPECTED_COUNTS:
            names = [item.get("name") for item in root.findall(".//" + tag)]
            self.assertEqual(len(names), len(set(names)))
            for name in names:
                self.assertRegex(name, r"^[A-Za-z][A-Za-z0-9_]{0,63}$")
        children = {
            joint.find("child").get("link") for joint in root.findall("joint")}
        self.assertEqual(
            {"base_link"},
            {link.get("name") for link in root.findall("link")} - children,
        )
        self.assertEqual(17, len(root.findall(".//visual")))
        self.assertEqual(17, len(root.findall(".//inertial")))
        self.assertNotIn(str(SOURCE_ROOT), payload)

    def test_runtime_joints_and_collision_are_exact(self):
        _, root = self._runtime()
        for joint in root.findall("joint"):
            self.assertEqual("fixed", joint.get("type"))
            for tag in INCOMPATIBLE_JOINT_TAGS:
                self.assertIsNone(joint.find(tag))
        collisions = root.findall(".//collision")
        self.assertEqual(1, len(collisions))
        collision = collisions[0]
        self.assertEqual("base_link_collision", collision.get("name"))
        self.assertEqual("base_link", next(
            link.get("name") for link in root.findall("link")
            if collision in list(link)))
        self.assertEqual(
            EXPECTED_BOX_ORIGIN, collision.find("origin").get("xyz"))
        self.assertEqual(
            EXPECTED_BOX_SIZE,
            collision.find("geometry/box").get("size"))
        self.assertIsNone(collision.find(".//mesh"))

    def test_lidar_and_planar_plugin_subtrees_are_exact(self):
        _, root = self._runtime()
        lidar_joint = next(
            item for item in root.findall("joint")
            if item.get("name") == "lidar_2d_joint")
        self.assertEqual(
            _signature(ET.fromstring(LIDAR_JOINT_XML)),
            _signature(lidar_joint),
        )
        lidar_gazebo = next(
            item for item in root.findall("gazebo")
            if item.get("reference") == "lidar_2d_link")
        planar_gazebo = next(
            item for item in root.findall("gazebo") if not item.attrib)
        self.assertEqual(
            _signature(ET.fromstring(LIDAR_GAZEBO_XML)),
            _signature(lidar_gazebo),
        )
        self.assertEqual(
            _signature(ET.fromstring(PLANAR_GAZEBO_XML)),
            _signature(planar_gazebo),
        )
        self.assertEqual(
            EXPECTED_PLUGIN_FILES,
            {item.get("filename") for item in root.findall(".//plugin")},
        )

    def test_runtime_validator_rejects_unsupported_mutations(self):
        payload, _ = self._runtime()

        def publish_tf(root):
            root.find(".//plugin").append(ET.Element("publishTF"))

        def tf_prefix(root):
            root.find(".//plugin").append(ET.Element("tf_prefix"))

        def prefixed_frame(root):
            root.find(".//frameName").text = "ground/lidar_2d_link"

        def unsupported_noise(root):
            root.find(".//plugin").append(ET.Element("gaussianNoise"))

        def forbidden_token(root):
            root.find(".//visual/geometry/mesh").set(
                "filename", "package://bunker_description/meshes/arm.STL")

        def third_plugin(root):
            root.find("gazebo").append(ET.Element(
                "plugin", {"name": "third", "filename": "third.so"}))

        for mutation in (
                publish_tf, tf_prefix, prefixed_frame, unsupported_noise,
                forbidden_token, third_plugin):
            root = ET.fromstring(payload)
            mutation(root)
            with self.subTest(mutation=mutation.__name__):
                with self.assertRaises(renderer.RenderContractError):
                    renderer.validate_runtime_tree(root)

    def test_render_is_byte_stable_hash_locked_and_check_urdf_valid(self):
        first = renderer.render_runtime_urdf(SOURCE)
        second = renderer.render_runtime_urdf(SOURCE)
        self.assertEqual(first, second)
        self.assertTrue(first.endswith("\n"))
        self.assertFalse(first.endswith("\n\n"))
        self.assertEqual(
            EXPECTED_RUNTIME_URDF_SHA256,
            hashlib.sha256(first.encode("utf-8")).hexdigest(),
        )
        output = Path(self.temporary.name) / "bunker.urdf"
        output.write_text(first, encoding="utf-8")
        result = subprocess.run(
            ["/usr/bin/check_urdf", str(output)],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False,
        )
        self.assertEqual(
            0, result.returncode,
            msg=result.stderr.decode("utf-8", "replace"),
        )

    def test_indent_fallback_has_frozen_three_level_whitespace(self):
        root = ET.fromstring("<root><one><two /></one></root>")
        renderer._indent_tree(root)
        self.assertEqual(
            "<root>\n  <one>\n    <two />\n  </one>\n</root>",
            ET.tostring(root, encoding="unicode"),
        )

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
