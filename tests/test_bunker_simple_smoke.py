#!/usr/bin/env python3

import importlib.util
import math
from pathlib import Path
import subprocess
from types import SimpleNamespace
import unittest


ROOT = Path(__file__).resolve().parents[1]
CHECKER = ROOT / "scripts/check_bunker_runtime.py"
SMOKE = ROOT / "scripts/smoke_bunker_standalone.bash"


def load_checker():
    if not CHECKER.is_file():
        raise AssertionError("the simple BUNKER runtime checker is missing")
    spec = importlib.util.spec_from_file_location("check_bunker_runtime", CHECKER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class BunkerSimpleSmokeTest(unittest.TestCase):
    def test_paper_evidence_pipeline_is_not_part_of_runtime(self):
        self.assertFalse((ROOT / "tools/bunker_live_contract.py").exists())
        self.assertFalse((ROOT / "scripts/probe_bunker_standalone.py").exists())
        self.assertFalse((ROOT / "scripts/start_bunker_runtime_group.py").exists())
        self.assertFalse((
            ROOT / "src/platform/bunker_sim_runtime/scripts/"
            "spawn_bunker_preflight.py").exists())

    def test_smoke_has_a_side_effect_free_help_command(self):
        completed = subprocess.run(
            [str(SMOKE), "--help"],
            cwd=ROOT,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=5,
            check=False,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("BUNKER standalone smoke", completed.stdout)
        self.assertIn("--run-root", completed.stdout)

    def test_yaw_and_local_motion_math(self):
        checker = load_checker()
        half = math.pi / 4.0
        yaw = checker.yaw_from_quaternion(0.0, 0.0, math.sin(half), math.cos(half))
        self.assertAlmostEqual(yaw, math.pi / 2.0)

        forward, lateral, turn = checker.relative_planar_motion(
            (2.0, 3.0, math.pi / 2.0),
            (2.0, 3.7, -math.pi),
        )
        self.assertAlmostEqual(forward, 0.7)
        self.assertAlmostEqual(lateral, 0.0)
        self.assertAlmostEqual(turn, math.pi / 2.0)

    def test_stamp_progress_requires_strictly_new_sensor_data(self):
        checker = load_checker()
        self.assertTrue(checker.stamp_advanced(12.0, 12.01))
        self.assertFalse(checker.stamp_advanced(12.0, 12.0))
        self.assertFalse(checker.stamp_advanced(12.0, 11.99))

    def test_imu_summary_requires_current_finite_ground_frame_data(self):
        checker = load_checker()
        message = SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(to_sec=lambda: 9.8),
                frame_id="ground/imu_link"),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            angular_velocity=SimpleNamespace(x=0.0, y=0.0, z=0.1),
            linear_acceleration=SimpleNamespace(x=0.0, y=0.0, z=9.81),
        )
        summary = checker.imu_summary(message, now=10.0)
        self.assertEqual("ground/imu_link", summary["frame"])
        self.assertAlmostEqual(0.2, summary["age_s"])

        message.angular_velocity.z = float("nan")
        with self.assertRaises(checker.RuntimeCheckError):
            checker.imu_summary(message, now=10.0)

    def test_status_summary_uses_official_bunker_boundary_shape(self):
        checker = load_checker()
        message = SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(to_sec=lambda: 9.9),
                frame_id="ground/base_link"),
            linear_velocity=0.2,
            angular_velocity=-0.1,
            base_state=0,
            control_mode=0,
            fault_code=0,
            battery_voltage=0.0,
        )
        summary = checker.bunker_status_summary(message, now=10.0)
        self.assertEqual("ground/base_link", summary["frame"])
        self.assertEqual(0, summary["fault_code"])
        self.assertAlmostEqual(0.2, summary["linear_velocity_mps"])

        message.linear_velocity = float("inf")
        with self.assertRaises(checker.RuntimeCheckError):
            checker.bunker_status_summary(message, now=10.0)

    def test_odometry_summary_uses_official_local_frames(self):
        checker = load_checker()
        vector = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        message = SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(to_sec=lambda: 9.9),
                frame_id="ground/odom"),
            child_frame_id="ground/base_link",
            pose=SimpleNamespace(pose=SimpleNamespace(
                position=vector,
                orientation=SimpleNamespace(
                    x=0.0, y=0.0, z=0.0, w=1.0))),
            twist=SimpleNamespace(twist=SimpleNamespace(
                linear=vector, angular=vector)),
        )
        summary = checker.odometry_summary(message, now=10.0)
        self.assertEqual("ground/odom", summary["frame"])
        self.assertEqual("ground/base_link", summary["child_frame"])
        message.child_frame_id = "world"
        with self.assertRaises(checker.RuntimeCheckError):
            checker.odometry_summary(message, now=10.0)

    def test_checker_uses_map_and_covers_driver_compatible_sensor_topics(self):
        source = CHECKER.read_text(encoding="utf-8")
        self.assertIn('Publisher("/ground/nav_cmd_vel"', source)
        self.assertIn('(\"map\", \"ground/base_link\")', source)
        self.assertIn('(\"ground/base_link\", \"ground/imu_link\")', source)
        self.assertIn('"/ground/imu/data"', source)
        self.assertIn('"/ground/bunker_status"', source)
        self.assertIn("from bunker_msgs.msg import BunkerStatus", source)
        self.assertIn("from sensor_msgs.msg import Imu, LaserScan", source)
        self.assertNotIn('(\"world\", \"ground/base_link\")', source)

    def test_model_check_waits_for_spawn_to_finish(self):
        checker = load_checker()
        responses = iter([
            SimpleNamespace(name=["ground_plane", "scan_obstacle"]),
            SimpleNamespace(name=["ground_plane", "scan_obstacle", "bunker"]),
        ])

        class FakeRospy:
            @staticmethod
            def wait_for_message(topic, _message_type, timeout):
                self.assertEqual(topic, "/gazebo/model_states")
                self.assertGreater(timeout, 0.0)
                return next(responses)

        self.assertTrue(checker.check_model(FakeRospy, object(), 1.0))

    def test_model_check_uses_the_available_startup_timeout(self):
        checker = load_checker()
        observed = []

        class FakeRospy:
            @staticmethod
            def wait_for_message(_topic, _message_type, timeout):
                observed.append(timeout)
                return SimpleNamespace(name=["bunker"])

        self.assertTrue(checker.check_model(FakeRospy, object(), 5.0))
        self.assertGreater(observed[0], 4.5)

    def test_scan_check_waits_past_startup_backlog_for_a_fresh_frame(self):
        checker = load_checker()

        def scan(stamp):
            return SimpleNamespace(
                header=SimpleNamespace(
                    stamp=SimpleNamespace(to_sec=lambda: stamp),
                    frame_id="ground/lidar_2d_link"),
                ranges=[2.0] * 720,
                range_min=0.12,
                range_max=8.0,
            )

        messages = iter([scan(1.0), scan(2.0), scan(9.5)])

        class FakeRospy:
            class Time:
                @staticmethod
                def now():
                    return SimpleNamespace(to_sec=lambda: 10.0)

            @staticmethod
            def wait_for_message(_topic, _message_type, timeout):
                self.assertGreater(timeout, 0.0)
                return next(messages)

        result = checker.check_scan(FakeRospy, object(), 1.0)
        self.assertAlmostEqual(result["age_s"], 0.5)
        self.assertEqual(result["frame"], "ground/lidar_2d_link")
        self.assertEqual(result["nearest_range_m"], 2.0)

    def test_scan_check_rejects_wrong_frame_and_self_return(self):
        checker = load_checker()

        def scan(stamp, frame, distance):
            return SimpleNamespace(
                header=SimpleNamespace(
                    stamp=SimpleNamespace(to_sec=lambda: stamp),
                    frame_id=frame),
                ranges=[distance] * 720,
                range_min=0.12,
                range_max=8.0,
            )

        for frame, distance in (
                ("lidar_2d_link", 2.0),
                ("ground/lidar_2d_link", 0.4)):
            messages = iter([
                scan(1.0, frame, distance),
                scan(2.0, frame, distance),
            ])

            class FakeRospy:
                class Time:
                    @staticmethod
                    def now():
                        return SimpleNamespace(to_sec=lambda: 2.0)

                @staticmethod
                def wait_for_message(_topic, _message_type, _timeout=None,
                                     **_kwargs):
                    return next(messages)

            with self.subTest(frame=frame, distance=distance):
                with self.assertRaises(checker.RuntimeCheckError):
                    checker.check_scan(FakeRospy, object(), 1.0)


if __name__ == "__main__":
    unittest.main()
