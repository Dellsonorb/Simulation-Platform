"""Exercise the installed SDFormat parser without starting Gazebo or ROS."""

import importlib.util
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import tempfile
import unittest
from unittest import mock
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src/platform/ground_manipulator_runtime"
PARSER_SOURCE = r"""
#include <iostream>
#include <iterator>
#include <string>
#include <sdf/sdf.hh>
int main() {
  const std::string xml((std::istreambuf_iterator<char>(std::cin)), {});
  sdf::SDFPtr doc(new sdf::SDF);
  sdf::init(doc);
  sdf::Errors errors;
  if (!sdf::readString(xml, doc, errors)) {
    for (const auto &error : errors) std::cerr << error.Message() << '\n';
    return 1;
  }
  std::cout << doc->ToString();
}
"""


class GroundSurfaceConversionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not shutil.which("g++") or not shutil.which("pkg-config"):
            raise unittest.SkipTest("C++ compiler and pkg-config are required")
        flags = subprocess.run(
            ["pkg-config", "--cflags", "--libs", "sdformat9"],
            capture_output=True, text=True, check=False)
        if flags.returncode:
            raise unittest.SkipTest("installed SDFormat 9 development package is required")
        cls.directory = tempfile.TemporaryDirectory(prefix="ground-surface-conversion-")
        cls.addClassCleanup(cls.directory.cleanup)
        source = Path(cls.directory.name) / "parse.cc"
        cls.converter = Path(cls.directory.name) / "parse"
        source.write_text(PARSER_SOURCE, encoding="utf-8")
        subprocess.run(["g++", "-std=c++17", str(source), "-o", str(cls.converter)]
                       + shlex.split(flags.stdout), check=True, capture_output=True, text=True)
        spec = importlib.util.spec_from_file_location(
            "ground_surface_renderer", PACKAGE / "src/ground_manipulator_runtime/renderer.py")
        renderer = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(renderer)
        package_path = os.pathsep.join((str(ROOT / "src"), "/opt/ros/noetic/share"))
        with mock.patch.dict(os.environ, {"ROS_PACKAGE_PATH": package_path}):
            cls.urdf = renderer.render_ground_robot(PACKAGE / "urdf/ground_robot.urdf.xacro")

    def convert(self, text):
        result = subprocess.run([str(self.converter)], input=text, text=True,
                                capture_output=True, check=True)
        return ET.fromstring(result.stdout)

    def base_box(self, model):
        base = model.find("model/link[@name='ground/base_link']")
        boxes = [collision for collision in base.findall("collision")
                 if collision.find("geometry/box") is not None]
        self.assertEqual(1, len(boxes))
        return boxes[0]

    def test_declared_base_friction_survives_real_fixed_joint_conversion(self):
        urdf = ET.fromstring(self.urdf)
        declared = urdf.find("gazebo[@reference='ground/base_link']")
        self.assertEqual("0.0", declared.findtext("mu1"))
        self.assertEqual("0.0", declared.findtext("mu2"))
        collision = self.base_box(self.convert(self.urdf))
        self.assertEqual("0", collision.findtext("surface/friction/ode/mu"))
        self.assertEqual("0", collision.findtext("surface/friction/ode/mu2"))

    def test_surface_transport_fix_preserves_geometry_inertia_and_joints(self):
        old_urdf = ET.fromstring(self.urdf)
        old_urdf.find("link[@name='ground/base_link']/collision").set("name", "base_link_collision")
        old = self.convert(ET.tostring(old_urdf, encoding="unicode"))
        current = self.convert(self.urdf)
        old_box, current_box = self.base_box(old), self.base_box(current)
        self.assertIsNone(old_box.find("surface"))
        for node in (old_box, current_box):
            node.attrib.pop("name")
            surface = node.find("surface")
            if surface is not None:
                node.remove(surface)
        # All other collision surfaces, shapes, masses, inertia and joint
        # definitions must remain exactly identical after conversion.
        for tree in (old, current):
            for element in tree.iter():
                if element.text is not None and not element.text.strip():
                    element.text = None
                element.tail = None
        self.assertEqual(ET.tostring(old), ET.tostring(current))


if __name__ == "__main__":
    unittest.main()
