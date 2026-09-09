"""Read-only physics trace regressions; no Gazebo world or ROS node is started."""

import csv
import math
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "src/platform/bunker_sim_runtime/src"
HEADER = SOURCE / "ground_dynamics_trace.hh"
PLUGIN = SOURCE / "bunker_planar_move_plugin.cpp"

HARNESS = r'''
#include "ground_dynamics_trace.hh"
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <string>
int main(int argc, char **argv) {
  using namespace gazebo::ground_dynamics;
  using ignition::math::Vector3d;
  const std::string mode = argv[1];
  if (mode == "off") {
    unsetenv("P450_GROUND_DYNAMICS_CSV");
    // With no opt-in even null physics inputs must not be dereferenced.
    Trace trace;
    assert(!trace.Open(gazebo::physics::ModelPtr(), gazebo::physics::LinkPtr(),
                       gazebo::physics::WorldPtr()));
    trace.Capture("update_end");
    trace.Close();
    CsvWriter writer;
    assert(!writer.OpenFromEnvironment());
    writer.Write(Snapshot(), 0.0, 0, "update_end");
    setenv("P450_GROUND_DYNAMICS_CSV", "", 1);
    assert(!writer.OpenFromEnvironment());
    setenv("P450_GROUND_DYNAMICS_CSV", "/nonexistent-parent/trace.csv", 1);
    assert(!writer.OpenFromEnvironment());
    writer.Close();
  } else if (mode == "projection") {
    const Vector3d axis(.6, .8, 0), parent(.3, -.2, .5);
    const Vector3d child = parent - axis * .16 + Vector3d(.8, -.6, 0) * .7;
    assert(std::abs(RelativeAngularRate(parent, child, axis) + .16) < 1e-12);
    assert(std::abs(RelativeAngularRate(parent, parent, axis)) < 1e-12);
    assert(std::abs(RelativeAngularRate(child, parent, axis) - .16) < 1e-12);
  } else if (mode == "csv") {
    assert(argc == 3);
    setenv("P450_GROUND_DYNAMICS_CSV", argv[2], 1);
    CsvWriter writer;
    assert(writer.OpenFromEnvironment());
    Snapshot state;
    state.joints[5].present = true;
    state.joints[5].position = .0123456789012345;
    state.joints[5].velocity = -.16;
    state.joints[5].force = 2.25;
    state.wrist_axis = Vector3d(.6, .8, 0);
    state.wrist_relative_rate = -.16;
    state.base.present = true;
    state.base.pose = ignition::math::Pose3d(1, 2, 3, 0, 0, 0);
    state.base.linear = Vector3d(4, 5, 6);
    state.base.angular = Vector3d(7, 8, 9);
    for (unsigned int i = 0; i < 300; ++i)
      writer.Write(state, 1.25 + i * .001, i, "before_base_set");
    // The fixed batch is visible before Close; not dependent on destruction.
    std::ifstream visible(argv[2]);
    std::string line;
    unsigned int lines = 0;
    while (std::getline(visible, line)) ++lines;
    assert(lines == 301);
    writer.Write(state, 2.0, 300, "after_base_set");
    writer.Write(state, 2.0, 300, "update_end");
    writer.Close();
    writer.Write(state, 9.0, 999, "update_end");
  }
}
'''


class GroundDynamicsTraceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory(prefix="ground-dynamics-trace-")
        cls.addClassCleanup(cls.directory.cleanup)
        cls.executable = Path(cls.directory.name) / "trace-test"

    def run_harness(self, *args):
        self.assertTrue(HEADER.exists(), "opt-in dynamics trace header is missing")
        if not self.executable.exists():
            flags = subprocess.check_output(
                ["pkg-config", "--cflags", "--libs", "gazebo"], text=True)
            compile_result = subprocess.run(
                ["g++", "-x", "c++", "-", "-o", str(self.executable),
                 "-I" + str(SOURCE)] + shlex.split(flags),
                input=HARNESS, capture_output=True, text=True)
            self.assertEqual(0, compile_result.returncode, compile_result.stderr)
        result = subprocess.run([str(self.executable)] + list(args),
                                capture_output=True, text=True)
        self.assertEqual(0, result.returncode, result.stderr)

    def test_default_off_empty_environment_and_bad_path_are_nonfatal(self):
        self.run_harness("off")

    def test_relative_angular_projection_subtracts_parent_in_world_frame(self):
        self.run_harness("projection")

    def test_csv_round_trip_availability_precision_and_batched_flush(self):
        path = Path(self.directory.name) / "trace.csv"
        self.run_harness("csv", str(path))
        with path.open() as stream:
            reader = csv.DictReader(stream)
            rows = list(reader)
            self.assertEqual(87, len(reader.fieldnames))
            self.assertEqual(len(reader.fieldnames), len(set(reader.fieldnames)))
        self.assertEqual(302, len(rows))
        self.assertTrue(all(None not in row for row in rows))
        self.assertEqual("before_base_set", rows[0]["phase"])
        self.assertEqual("after_base_set", rows[-2]["phase"])
        self.assertEqual("update_end", rows[-1]["phase"])
        self.assertEqual("300", rows[-1]["iteration"])
        self.assertEqual(1.25, float(rows[0]["sim_time_s"]))
        self.assertEqual(.0123456789012345, float(rows[0]["wrist_3_joint_q_rad"]))
        self.assertEqual(-.16, float(rows[0]["wrist_3_joint_dq_rad_s"]))
        self.assertEqual(2.25, float(rows[0]["wrist_3_joint_force_native"]))
        self.assertEqual("1", rows[0]["base_present"])
        for suffix, expected in (("x_m", 1), ("y_m", 2), ("z_m", 3),
                                 ("qw", 1), ("qx", 0), ("qy", 0), ("qz", 0),
                                 ("vx_m_s", 4), ("vy_m_s", 5), ("vz_m_s", 6),
                                 ("wx_rad_s", 7), ("wy_rad_s", 8), ("wz_rad_s", 9)):
            self.assertEqual(expected, float(rows[0]["base_" + suffix]))
        for prefix in ("wrist_parent", "wrist_child", "target"):
            self.assertEqual("0", rows[0][prefix + "_present"])
            self.assertTrue(math.isnan(float(rows[0][prefix + "_x_m"])))
        self.assertEqual("0", rows[0]["shoulder_pan_joint_present"])
        self.assertTrue(math.isnan(float(rows[0]["shoulder_pan_joint_q_rad"])))

    def test_snapshot_collector_has_only_read_only_physics_calls(self):
        self.assertTrue(HEADER.exists(), "opt-in dynamics trace header is missing")
        source = HEADER.read_text()
        self.assertNotRegex(source, r"(?:->|\.)\s*(?:Set\w*|Add\w*|Reset|Init|Load)\s*\(")
        for getter in ("Position(0)", "GetVelocity(0)", "GetForce(0)",
                       "GlobalAxis(0)", "GetParent()", "GetChild()",
                       "WorldPose()", "WorldLinearVel()", "WorldAngularVel()"):
            self.assertIn(getter, source)
        self.assertIn('ModelByName("pick_target")', source)
        self.assertNotIn("std::endl", source)

    def test_plugin_brackets_only_base_setters_and_disconnects_before_close(self):
        source = PLUGIN.read_text()
        self.assertIn('dynamics_trace_.Capture("before_base_set")', source)
        before = source.index('dynamics_trace_.Capture("before_base_set")')
        linear = source.index("base_link_->SetLinearVel", before)
        angular = source.index("base_link_->SetAngularVel", linear)
        after = source.index('dynamics_trace_.Capture("after_base_set")', angular)
        self.assertLess(before, linear)
        self.assertLess(linear, angular)
        self.assertLess(angular, after)
        self.assertIn("ConnectWorldUpdateEnd", source)
        self.assertIn('dynamics_trace_.Capture("update_end")', source)
        self.assertRegex(source, r"if \(dynamics_trace_\.Open\(model_, base_link_, world_\)\)")
        shutdown = source[source.index("  void Shutdown() {"):source.index("  void RunCallbackQueue()")]
        self.assertLess(shutdown.index("update_end_connection_.reset()"),
                        shutdown.index("dynamics_trace_.Close()"))
        self.assertLess(shutdown.index("update_connection_.reset()"),
                        shutdown.index("dynamics_trace_.Close()"))


if __name__ == "__main__":
    unittest.main()
