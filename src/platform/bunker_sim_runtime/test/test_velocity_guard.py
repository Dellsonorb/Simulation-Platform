import importlib.util
import sys
import unittest
from pathlib import Path
from unittest import mock

from bunker_sim_runtime.velocity_guard import ZERO, admit_components


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = PACKAGE_ROOT / "src/bunker_sim_runtime/velocity_guard.py"

VALID_CASES = (
    ((0.25, 0.0, 0.0, 0.0, 0.0, 0.5),
     (0.25, 0.0, 0.0, 0.0, 0.0, 0.5), None),
    ((0.75, 0.0, 0.0, 0.0, 0.0, 2.0),
     (0.5, 0.0, 0.0, 0.0, 0.0, 1.0), "clamped"),
    ((-0.75, 0.0, 0.0, 0.0, 0.0, -2.0),
     (-0.5, 0.0, 0.0, 0.0, 0.0, -1.0), "clamped"),
)

REJECT_CASES = (
    ((float("nan"), 0, 0, 0, 0, 0), "nonfinite"),
    ((0, 0, 0, 0, 0, float("inf")), "nonfinite"),
    ((0, 1.1e-9, 0, 0, 0, 0), "nonplanar"),
    ((0, 0, -1.1e-9, 0, 0, 0), "nonplanar"),
    ((0, 0, 0, 1.1e-9, 0, 0), "nonplanar"),
    ((0, 0, 0, 0, -1.1e-9, 0), "nonplanar"),
)


class VelocityGuardTest(unittest.TestCase):
    def test_admits_and_clamps_planar_components(self):
        for raw, expected, reason in VALID_CASES:
            with self.subTest(raw=raw):
                self.assertEqual((expected, reason), admit_components(*raw))

    def test_rejects_nonfinite_and_nonplanar_components(self):
        for raw, reason in REJECT_CASES:
            with self.subTest(raw=raw):
                self.assertEqual((ZERO, reason), admit_components(*raw))

    def test_epsilon_boundary_is_normalized_to_zero(self):
        raw = (0.2, 1e-9, -1e-9, 1e-9, -1e-9, 0.3)
        self.assertEqual(
            ((0.2, 0.0, 0.0, 0.0, 0.0, 0.3), None),
            admit_components(*raw),
        )

    def test_boolean_and_nonnumeric_values_are_nonfinite(self):
        for value in (True, False, object(), "not-a-number"):
            raw = [0.0] * 6
            raw[0] = value
            with self.subTest(value=value):
                self.assertEqual((ZERO, "nonfinite"), admit_components(*raw))

    def test_input_container_is_not_mutated(self):
        raw = [0.75, 0.0, 0.0, 0.0, 0.0, 2.0]
        snapshot = list(raw)
        admit_components(*raw)
        self.assertEqual(snapshot, raw)

    def test_pure_module_imports_without_ros_modules(self):
        name = "bunker_velocity_guard_no_ros_fixture"
        spec = importlib.util.spec_from_file_location(name, str(MODULE_PATH))
        module = importlib.util.module_from_spec(spec)
        with mock.patch.dict(
                sys.modules, {"rospy": None, "geometry_msgs": None}):
            spec.loader.exec_module(module)
        self.assertTrue(callable(module.admit_components))


if __name__ == "__main__":
    unittest.main()
