#!/usr/bin/env python3
"""Unit and source-boundary tests for the thin Prometheus facade."""

import importlib.util
import math
from pathlib import Path
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
MODULE = PACKAGE / "src/p450_flight_facade/translation.py"
SERVER = PACKAGE / "scripts/p450_flight_facade_node.py"


def _load_translation():
    if not MODULE.is_file():
        return None
    spec = importlib.util.spec_from_file_location("translation", str(MODULE))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class PrometheusTranslationTest(unittest.TestCase):
    def setUp(self):
        self.translation = _load_translation()
        self.assertIsNotNone(self.translation, MODULE)

    def test_takeoff_is_only_the_native_prometheus_sequence(self):
        self.assertEqual((
            ("setup", "ARMING", True),
            ("setup", "SET_CONTROL_MODE", "COMMAND_CONTROL"),
            ("setup", "SET_PX4_MODE", "OFFBOARD"),
            ("command", "Init_Pos_Hover", None),
        ), self.translation.translation_for(self.translation.TAKEOFF))

    def test_fly_to_hover_land_and_cancel_are_direct_translations(self):
        target = (1.25, -0.5, 2.0, math.pi / 3.0)
        self.assertEqual((
            ("command", "Move_XYZ_POS", target),
        ), self.translation.translation_for(self.translation.FLY_TO, target))
        self.assertEqual((
            ("command", "Current_Pos_Hover", None),
        ), self.translation.translation_for(self.translation.HOVER))
        self.assertEqual((
            ("command", "Land", None),
        ), self.translation.translation_for(self.translation.LAND))
        self.assertEqual(
            self.translation.translation_for(self.translation.HOVER),
            self.translation.preempt_translation(self.translation.FLY_TO))

    def test_translation_rejects_unknown_or_invalid_targets(self):
        with self.assertRaises(self.translation.TranslationError):
            self.translation.translation_for(0)
        with self.assertRaises(self.translation.TranslationError):
            self.translation.translation_for(self.translation.FLY_TO)
        with self.assertRaises(self.translation.TranslationError):
            self.translation.translation_for(
                self.translation.FLY_TO, (0.0, float("nan"), 1.0, 0.0))
        with self.assertRaises(self.translation.TranslationError):
            self.translation.translation_for(
                self.translation.HOVER, (0.0, 0.0, 1.0, 0.0))

    def test_native_health_and_completion_are_observations_not_control(self):
        self.assertTrue(self.translation.backend_healthy(
            connected=True, odom_valid=True, armed=True, failsafe=False,
            state_age=0.1, control_age=0.1, max_age=0.5))
        self.assertFalse(self.translation.backend_healthy(
            connected=True, odom_valid=True, armed=True, failsafe=True,
            state_age=0.1, control_age=0.1, max_age=0.5))
        self.assertTrue(self.translation.position_complete(
            (1.0, 2.0, 3.0), (1.02, 1.98, 3.01), (0.01, 0.0, 0.0),
            position_tolerance=0.05, settle_speed=0.05))
        self.assertFalse(self.translation.position_complete(
            (1.0, 2.0, 3.0), (1.2, 2.0, 3.0), (0.0, 0.0, 0.0),
            position_tolerance=0.05, settle_speed=0.05))


class FacadeSourceBoundaryTest(unittest.TestCase):
    def test_installed_node_does_not_shadow_the_python_package(self):
        self.assertNotEqual(PACKAGE.name + ".py", SERVER.name)
        self.assertTrue(SERVER.is_file(), SERVER)

    def test_server_uses_only_native_prometheus_backend_topics(self):
        self.assertTrue(SERVER.is_file(), SERVER)
        text = SERVER.read_text(encoding="utf-8")
        for required in (
                "UAVCommand", "UAVControlState", "UAVSetup", "UAVState",
                "/uav1/prometheus/command", "/uav1/prometheus/setup",
                "/uav1/prometheus/state", "/uav1/prometheus/control_state",
                "/uav1/prometheus/odom", "/uav1/runtime/flight"):
            self.assertIn(required, text)
        for forbidden in (
                "mavros_msgs", "gazebo_msgs", "setpoint_raw",
                "setpoint_position", "class FlightState", "rospy.Timer"):
            self.assertNotIn(forbidden, text)


if __name__ == "__main__":
    unittest.main()
