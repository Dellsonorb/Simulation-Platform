"""Compiled numeric and wiring checks; never start ROS or a Gazebo world."""
import subprocess
import tempfile
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'src/platform/ground_manipulator_runtime'
HEADER = PACKAGE / 'src/integrated_joint_velocity.hh'
SOURCE = PACKAGE / 'src/integrated_velocity_robot_hw_sim.cpp'

HARNESS = r'''
#include "integrated_joint_velocity.hh"
#include <cassert>
#include <cmath>
#include <limits>
#include <string>
int main(int, char **argv) {
  using ground_manipulator_runtime::IntegratedJointVelocity;
  using ground_manipulator_runtime::IntegratedVelocityEnabled;
  const std::string mode = argv[1];
  const double nan = std::numeric_limits<double>::quiet_NaN();
  IntegratedJointVelocity sample;
  double velocity = -.16;
  if (mode == "enable") {
    assert(!IntegratedVelocityEnabled(nullptr));
    for (const char *value : {"", "0", "true", "01", "1 "})
      assert(!IntegratedVelocityEnabled(value));
    assert(IntegratedVelocityEnabled("1"));
  } else if (mode == "motion") {
    assert(!sample.Update(0., 3.14, velocity));
    assert(velocity == -.16);
    // Inputs are already unwrapped by stock readSim: preserve the pi crossing.
    assert(sample.Update(.001, 3.141, velocity));
    assert(std::abs(velocity - 1.) < 1e-9);
    assert(sample.Update(.004, 3.147, velocity));
    assert(std::abs(velocity - 2.) < 1e-9);
    assert(sample.Update(.006, 3.141, velocity));
    assert(std::abs(velocity + 3.) < 1e-9);
  } else if (mode == "stationary_bias") {
    assert(!sample.Update(0., .7, velocity));
    for (int i = 1; i <= 2000; ++i) {
      velocity = -.16;  // Stock native rate remains available independently.
      assert(sample.Update(i * .001, .7, velocity));
      assert(velocity == 0.);
    }
  } else if (mode == "alternation") {
    assert(!sample.Update(0., 0., velocity));
    double integrated = 0.;
    for (int i = 1; i <= 2000; ++i) {
      const double position = i % 2 ? .0002 : 0.;
      assert(sample.Update(i * .001, position, velocity));
      assert(std::abs(velocity - (i % 2 ? .2 : -.2)) < 1e-9);
      integrated += velocity * .001;
    }
    assert(std::abs(integrated) < 1e-10);
  } else if (mode == "guards") {
    assert(!sample.Update(10., 1., velocity));
    assert(!sample.Update(10., 1.1, velocity));
    assert(velocity == -.16);
    assert(sample.Update(10.001, 1.101, velocity));
    assert(std::abs(velocity - 1.) < 1e-9);
    velocity = -.16;
    assert(!sample.Update(0., 2., velocity));
    assert(velocity == -.16);
    assert(sample.Update(.001, 2.002, velocity));
    assert(std::abs(velocity - 2.) < 1e-9);
    for (bool bad_time : {false, true}) {
      velocity = -.16;
      assert(!sample.Update(bad_time ? nan : .002, bad_time ? 2. : nan, velocity));
      assert(velocity == -.16);
      assert(!sample.Update(.003, 2., velocity));
      assert(velocity == -.16);
      assert(sample.Update(.004, 2.001, velocity));
      assert(std::abs(velocity - 1.) < 1e-9);
    }
  } else if (mode == "overflow") {
    const double largest = std::numeric_limits<double>::max();
    assert(!sample.Update(0., -largest, velocity));
    assert(!sample.Update(.001, largest, velocity));
    assert(velocity == -.16);
  } else if (mode == "invalid_native") {
    assert(!sample.Update(0., 0., velocity));
    velocity = nan;
    assert(!sample.Update(.001, .001, velocity));
    assert(std::isnan(velocity));
    velocity = -.16;
    assert(!sample.Update(.002, .002, velocity));
    assert(velocity == -.16);
    assert(sample.Update(.003, .003, velocity));
    assert(std::abs(velocity - 1.) < 1e-9);
  }
}
'''


class IntegratedJointVelocityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory(prefix='integrated-joint-velocity-')
        cls.addClassCleanup(cls.directory.cleanup)
        cls.executable = Path(cls.directory.name) / 'numeric-test'

    def run_harness(self, mode):
        self.assertTrue(HEADER.is_file(), 'integrated velocity helper is missing')
        if not self.executable.exists():
            result = subprocess.run(
                ['g++', '-std=c++14', '-Wall', '-Wextra', '-Werror', '-x', 'c++', '-',
                 '-I' + str(HEADER.parent), '-o', str(self.executable)],
                input=HARNESS, text=True, capture_output=True)
            self.assertEqual(0, result.returncode, result.stderr)
        result = subprocess.run([str(self.executable), mode], text=True, capture_output=True)
        self.assertEqual(0, result.returncode, result.stderr)

    def test_only_exact_one_enables_feedback(self): self.run_harness('enable')
    def test_nonzero_unwrapped_motion_uses_actual_interval(self): self.run_harness('motion')
    def test_stationary_position_despite_native_bias(self): self.run_harness('stationary_bias')
    def test_every_500hz_alternating_increment_is_retained(self): self.run_harness('alternation')
    def test_first_reset_duplicate_and_nonfinite_preserve_native(self): self.run_harness('guards')
    def test_nonfinite_difference_preserves_native(self): self.run_harness('overflow')
    def test_nonfinite_native_rate_is_not_concealed(self): self.run_harness('invalid_native')

    def test_plugin_wiring_and_stock_write_path(self):
        self.assertTrue(SOURCE.is_file(), 'SIM hardware feedback plugin is missing')
        source = SOURCE.read_text()
        self.assertIn('public gazebo_ros_control::DefaultRobotHWSim', source)
        self.assertIn('DefaultRobotHWSim::readSim(time, period)', source)
        self.assertIn('P450_GROUND_INTEGRATED_VELOCITY', source)
        self.assertNotIn('void writeSim', source)
        self.assertNotRegex(source, r'joint_velocity_\.(?:resize|swap|assign)')
        self.assertNotRegex(source, r'->(?:Set|Add)\w+\(')
        self.assertIn('PLUGINLIB_EXPORT_CLASS', source)
        root = ET.parse(PACKAGE / 'robot_hw_sim_plugins.xml').getroot()
        plugin = root.find('class')
        self.assertEqual('gazebo_ros_control::RobotHWSim', plugin.get('base_class_type'))
        robot = ET.parse(PACKAGE / 'urdf/ground_robot.urdf.xacro').getroot()
        control = robot.find("./gazebo/plugin[@name='gazebo_ros_control']")
        self.assertEqual(plugin.get('name'), control.findtext('robotSimType'))
        manifest = ET.parse(PACKAGE / 'package.xml').getroot()
        self.assertEqual('${prefix}/robot_hw_sim_plugins.xml',
                         manifest.find('export/gazebo_ros_control').get('plugin'))
        cmake = (PACKAGE / 'CMakeLists.txt').read_text()
        self.assertIn('integrated_velocity_robot_hw_sim.cpp', cmake)
        self.assertIn('robot_hw_sim_plugins.xml', cmake)


if __name__ == '__main__': unittest.main()
