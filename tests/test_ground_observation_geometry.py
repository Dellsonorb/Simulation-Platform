import importlib.util
from pathlib import Path
import sys
import unittest
import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]/'src/demos/air_ground_pick_demo/src'
sys.path.insert(0, str(PACKAGE))


class CameraObservationTests(unittest.TestCase):
    def setUp(self):
        from air_ground_pick_demo import ground_observation as module
        self.module = module
        self.extrinsic = np.array([[0, 0, 1, -.095], [-1, 0, 0, 0],
                                   [0, -1, 0, .075], [0, 0, 0, 1.]])
        self.k = np.array([[462.14, 0, 320.5], [0, 462.14, 240.5], [0, 0, 1.]])

    def test_camera_not_tcp_is_centered_over_measured_target(self):
        target = [1., 2., .0575, .3]
        views = self.module.observation_poses(target, [.24, .053, .115], self.extrinsic)
        self.assertEqual(len(views), 6)
        for matrix in views:
            camera = matrix @ self.extrinsic
            np.testing.assert_allclose(camera[:2, 3], target[:2], atol=1e-12)
            np.testing.assert_allclose(camera[:3, 2], [0, 0, -1], atol=1e-12)
            self.assertTrue(self.module.target_in_fov(camera, target, [.24, .053, .115], self.k, [640, 480]))
        self.assertGreater(np.linalg.norm(views[0][:2, 3]-target[:2]), .07)

    def test_fov_rejects_offscreen_or_behind_camera(self):
        camera = np.eye(4)
        self.assertFalse(self.module.target_in_fov(camera, [0, 0, -1, 0], [.24, .053, .115], self.k, [640, 480]))
        self.assertFalse(self.module.target_in_fov(camera, [2, 0, .3, 0], [.24, .053, .115], self.k, [640, 480]))

    def test_nonfinite_mount_not_plannable(self):
        self.extrinsic[0, 3] = np.nan
        with self.assertRaises(ValueError):
            self.module.observation_poses([0, 0, .0575, 0], [.24, .053, .115], self.extrinsic)

    def test_mount_rotation_is_composed_not_assumed(self):
        target = [0., 0., .0575, .2]
        mount = np.eye(4)
        mount[:3, 3] = [.1, 0, .05]
        for tcp in self.module.observation_poses(target, [.24, .053, .115], mount):
            camera = tcp @ mount
            np.testing.assert_allclose(camera[:3, 2], [0, 0, -1], atol=1e-12)


if __name__ == '__main__': unittest.main()
