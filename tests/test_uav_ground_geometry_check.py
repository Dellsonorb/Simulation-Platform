import importlib.util
from pathlib import Path
import unittest
import numpy as np

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/check_uav_map_ground.py'


class GroundGeometryCheckTests(unittest.TestCase):
    def setUp(self):
        spec = importlib.util.spec_from_file_location('ground_check', SCRIPT)
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)

    def test_passes_physical_ground_with_nonzero_sensor_height(self):
        points = np.array([[7, 2, -2.], [8, 2, -1.995], [9, 2, -2.005]])
        transform = np.eye(4)
        transform[2, 3] = 2
        result = self.module.ground_summary(points, transform, (6, 10, 1.5, 4))
        self.assertTrue(result['within_tolerance'])
        self.assertAlmostEqual(result['max_abs_error_m'], .005)

    def test_detects_old_bias_without_subtracting_plane_or_changing_ground(self):
        result = self.module.ground_summary(np.array([[7, 2, .10]]), np.eye(4), (6, 10, 1.5, 4))
        self.assertFalse(result['within_tolerance'])
        self.assertAlmostEqual(result['median_error_m'], .10)

    def test_no_roi_returns_does_not_pass_vacuously(self):
        with self.assertRaisesRegex(ValueError, 'ground patch'):
            self.module.ground_summary(np.array([[1, 0, 0]]), np.eye(4), (6, 10, 1.5, 4))


if __name__ == '__main__':
    unittest.main()
