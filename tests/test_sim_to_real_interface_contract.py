#!/usr/bin/env python3
"""Static contracts for the SIM/REAL-common robot-facing messages."""

from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
INTERFACES = ROOT / "src/platform/robot_runtime_interfaces"
BUNKER_MSGS = ROOT / "src/vendor/bunker_msgs"


def _meaningful_lines(path):
    return [
        line.strip()
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]


class FlightCommandContractTest(unittest.TestCase):
    def test_action_has_only_the_frozen_minimum_contract(self):
        action = INTERFACES / "action/FlightCommand.action"
        self.assertTrue(action.is_file(), action)
        sections = re.split(r"^---\s*$", action.read_text(encoding="utf-8"),
                            flags=re.MULTILINE)
        self.assertEqual(3, len(sections))
        parsed = [
            [line for line in section.splitlines()
             if line.strip() and not line.lstrip().startswith("#")]
            for section in sections
        ]
        self.assertEqual([
            "uint8 TAKEOFF=1",
            "uint8 FLY_TO=2",
            "uint8 HOVER=3",
            "uint8 LAND=4",
            "uint8 command",
            "geometry_msgs/PoseStamped target",
        ], parsed[0])
        self.assertEqual(["bool success", "string message"], parsed[1])
        self.assertEqual([
            "geometry_msgs/PoseStamped current_pose",
            "float32 position_error",
            "string native_mode",
        ], parsed[2])

    def test_interface_package_generates_an_action_without_sim_dependencies(self):
        cmake = INTERFACES / "CMakeLists.txt"
        package = INTERFACES / "package.xml"
        self.assertTrue(cmake.is_file(), cmake)
        self.assertTrue(package.is_file(), package)
        text = cmake.read_text(encoding="utf-8") + package.read_text(
            encoding="utf-8")
        for dependency in (
                "actionlib_msgs", "geometry_msgs", "message_generation",
                "message_runtime"):
            self.assertIn(dependency, text)
        for forbidden in ("gazebo", "mavros", "prometheus_msgs",
                          "RobotState.msg"):
            self.assertNotIn(forbidden, text)


class BunkerMessageContractTest(unittest.TestCase):
    def test_status_matches_the_official_ros1_fields(self):
        status = BUNKER_MSGS / "msg/BunkerStatus.msg"
        motor = BUNKER_MSGS / "msg/BunkerMotorState.msg"
        self.assertTrue(status.is_file(), status)
        self.assertTrue(motor.is_file(), motor)
        self.assertEqual([
            "Header header",
            "int8 MOTOR_ID_FRONT_RIGHT = 0",
            "int8 MOTOR_ID_FRONT_LEFT = 1",
            "int8 MOTOR_ID_REAR_RIGHT = 2",
            "int8 MOTOR_ID_REAR_LEFT = 3",
            "int8 LIGHT_ID_FRONT = 0",
            "int8 LIGHT_ID_REAR = 1",
            "float64 linear_velocity",
            "float64 angular_velocity",
            "uint8 base_state",
            "uint8 control_mode",
            "uint16 fault_code",
            "float64 battery_voltage",
            "BunkerMotorState[3] motor_states",
        ], _meaningful_lines(status))
        self.assertEqual([
            "float64 current",
            "float64 rpm",
            "float64 temperature",
        ], _meaningful_lines(motor))

    def test_message_package_is_message_only(self):
        cmake = BUNKER_MSGS / "CMakeLists.txt"
        package = BUNKER_MSGS / "package.xml"
        self.assertTrue(cmake.is_file(), cmake)
        self.assertTrue(package.is_file(), package)
        text = cmake.read_text(encoding="utf-8") + package.read_text(
            encoding="utf-8")
        for dependency in ("std_msgs", "message_generation",
                           "message_runtime"):
            self.assertIn(dependency, text)
        for forbidden in ("gazebo", "ugv_sdk", "socketcan", "can_msgs",
                          "RobotState.msg"):
            self.assertNotIn(forbidden, text)


if __name__ == "__main__":
    unittest.main()
