#!/usr/bin/env python3
"""Static contracts for the SIM/REAL-common robot-facing messages."""

from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
INTERFACES = ROOT / "src/platform/robot_runtime_interfaces"
BUNKER_MSGS = ROOT / "src/vendor/bunker_msgs"
SIM_BRINGUP = ROOT / "src/platform/sim_platform_bringup"
SIM_COMPOSITION = SIM_BRINGUP / "launch/air_ground_standalone.launch"
DEMO_LAUNCH = (
    ROOT / "src/demos/air_ground_pick_demo/launch/air_ground_pick_demo.launch")
RUNTIME_CHECKER = ROOT / "scripts/check_air_ground_runtime.py"
BUNKER_CHECKER = ROOT / "scripts/check_bunker_runtime.py"
PICK_CHECKER = ROOT / "scripts/check_air_ground_pick_demo.py"
PLATFORM_SMOKE = ROOT / "scripts/smoke_air_ground_standalone.bash"
DEMO_SMOKE = ROOT / "scripts/smoke_air_ground_pick_demo.bash"
README = ROOT / "README.md"


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


class CommonRuntimeCompositionTest(unittest.TestCase):
    def test_sim_composition_starts_one_common_flight_and_navigation_layer(self):
        sim_includes = [
            item.get("file")
            for item in ET.parse(str(SIM_COMPOSITION)).getroot().findall(
                "include")]
        self.assertEqual(1, sum(
            "p450_flight_facade.launch" in item for item in sim_includes))
        self.assertEqual(1, sum(
            "ground_navigation.launch" in item for item in sim_includes))

        demo_includes = [
            item.get("file")
            for item in ET.parse(str(DEMO_LAUNCH)).getroot().findall("include")]
        self.assertFalse(any(
            "p450_flight_facade.launch" in item for item in demo_includes))
        self.assertFalse(any(
            "ground_navigation.launch" in item for item in demo_includes))

        dependencies = {
            item.text
            for item in ET.parse(str(SIM_BRINGUP / "package.xml"))
            .getroot().findall("exec_depend")}
        self.assertIn("p450_flight_facade", dependencies)
        self.assertIn("bunker_navigation", dependencies)

    def test_joint_checker_probes_common_and_native_robot_interfaces(self):
        source = RUNTIME_CHECKER.read_text(encoding="utf-8")
        combined = source + BUNKER_CHECKER.read_text(encoding="utf-8")
        for required in (
                "FlightCommandAction", "MoveBaseAction",
                '"/uav1/runtime/flight"', '"/ground/move_base"',
                '"/ground/runtime/stop"',
                '"/uav1/prometheus/state"',
                '"/uav1/prometheus/odom"',
                '"/uav1/livox/lidar"', "PointCloud2",
                '"/ground/odom"', '"/ground/bunker_status"',
                '"/ground/scan"', '"/ground/imu/data"'):
            self.assertIn(required, combined)

    def test_pick_checker_uses_map_and_keeps_gazebo_as_external_oracle(self):
        source = PICK_CHECKER.read_text(encoding="utf-8")
        self.assertIn('parser.add_argument("--map-frame", default="map")', source)
        self.assertIn("args.map_frame", source)
        self.assertNotIn("args.world_frame", source)
        self.assertIn("Use Gazebo pose only as an external E2E test oracle", source)
        self.assertIn('default="/ground/gripper/grasp_confirmed"', source)

    def test_full_platform_smokes_enable_the_standard_mid360_topic(self):
        for path in (PLATFORM_SMOKE, DEMO_SMOKE):
            source = path.read_text(encoding="utf-8")
            self.assertIn("enable_mid360:=true", source, path.name)

    def test_readme_records_public_contract_and_real_replacement_boundaries(self):
        source = README.read_text(encoding="utf-8")
        for required in (
                "## SIM/REAL robot-facing contract",
                "/uav1/runtime/flight",
                "/ground/move_base",
                "/ground/runtime/stop",
                "/ground/gripper/grasp_confirmed",
                "map -> uav1/odom -> uav1/base_link",
                "map -> ground/odom -> ground/base_link",
                "p450_experiment + Prometheus + MAVROS + PX4",
                "bunker_base -> ugv_sdk -> CAN"):
            self.assertIn(required, source)


if __name__ == "__main__":
    unittest.main()
