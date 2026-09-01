#!/usr/bin/env python3

import importlib.util
import math
import os
from pathlib import Path
import subprocess
import unittest
import warnings
import xml.etree.ElementTree as ET

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src/demos/air_ground_pick_demo"
PACKAGE_XML = PACKAGE / "package.xml"
CMAKE = PACKAGE / "CMakeLists.txt"
PERCEPTION = (
    PACKAGE / "src/air_ground_pick_demo/perception.py")
FLIGHT = PACKAGE / "src/air_ground_pick_demo/flight.py"
APPROACH = PACKAGE / "src/air_ground_pick_demo/approach.py"
TARGET_MODEL = PACKAGE / "models/pick_target/model.sdf"
TARGET_CONFIG = PACKAGE / "models/pick_target/model.config"
OBSERVER = PACKAGE / "scripts/red_target_observer.py"
AIR_CONFIG = PACKAGE / "config/air_observer.yaml"
GROUND_CONFIG = PACKAGE / "config/ground_observer.yaml"
OBSERVERS_LAUNCH = PACKAGE / "launch/target_observers.launch"
ORCHESTRATOR = PACKAGE / "scripts/air_ground_pick_demo.py"
DEMO_CONFIG = PACKAGE / "config/demo.yaml"


def _load_perception():
    if not PERCEPTION.is_file():
        raise AssertionError("minimal pick perception module is missing")
    spec = importlib.util.spec_from_file_location(
        "air_ground_pick_perception_test_target", str(PERCEPTION))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_module(path, module_name):
    if not path.is_file():
        raise AssertionError("%s is missing" % path.name)
    spec = importlib.util.spec_from_file_location(module_name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class MinimalAirGroundPickDemoTest(unittest.TestCase):
    def test_package_is_a_small_sim_demo_layer(self):
        root = ET.parse(str(PACKAGE_XML)).getroot()
        self.assertEqual("air_ground_pick_demo", root.findtext("name"))
        dependencies = {
            item.text for tag in ("build_depend", "exec_depend")
            for item in root.findall(tag)}
        for required in (
                "cv_bridge", "geometry_msgs", "message_filters", "rospy",
                "sensor_msgs", "tf2_ros"):
            self.assertIn(required, dependencies)
        cmake = CMAKE.read_text(encoding="utf-8")
        self.assertIn("catkin_python_setup", cmake)
        self.assertIn("DIRECTORY models", cmake)
        combined = cmake.lower() + PACKAGE_XML.read_text(
            encoding="utf-8").lower()
        for forbidden in (
                "benchmark", "provenance", "artifact", "pilot", "formal"):
            self.assertNotIn(forbidden, combined)

    def test_pick_target_is_dynamic_contact_sensed_and_gripper_feasible(self):
        model = ET.parse(str(TARGET_MODEL)).getroot().find("model")
        self.assertIsNotNone(model)
        self.assertEqual("pick_target", model.get("name"))
        self.assertNotEqual("true", model.findtext("static", "false"))
        link = model.find("link")
        self.assertEqual("pick_target_link", link.get("name"))
        self.assertAlmostEqual(0.20, float(link.findtext("inertial/mass")))
        inertia = link.find("inertial/inertia")
        for name in ("ixx", "iyy", "izz"):
            self.assertGreater(float(inertia.findtext(name)), 0.0)
        collision = link.find("collision")
        size = tuple(float(value) for value in
                     collision.findtext("geometry/box/size").split())
        self.assertEqual((0.240, 0.053, 0.115), size)
        self.assertLess(size[1] + 0.002, 0.0952)
        surface = collision.find("surface")
        self.assertGreaterEqual(float(surface.findtext("friction/ode/mu")), 1.0)
        self.assertGreaterEqual(float(surface.findtext("friction/ode/mu2")), 1.0)
        material = link.find("visual/material")
        self.assertEqual(
            "Gazebo/Red", material.findtext("script/name"))
        self.assertEqual(
            "file://media/materials/scripts/gazebo.material",
            material.findtext("script/uri"))
        sensor = link.find("sensor[@type='contact']")
        self.assertIsNotNone(sensor)
        self.assertEqual(
            collision.get("name"), sensor.findtext("contact/collision"))
        plugin = sensor.find("plugin")
        self.assertEqual("libgazebo_ros_bumper.so", plugin.get("filename"))
        self.assertEqual("/pick_target/contacts",
                         plugin.findtext("bumperTopicName"))
        self.assertTrue(TARGET_CONFIG.is_file())

    def test_red_component_selection_rejects_ambiguity(self):
        perception = _load_perception()
        image = np.zeros((100, 120, 3), dtype=np.uint8)
        image[30:70, 40:90, 0] = 230
        mask = perception.select_red_component(
            image, min_pixels=500, ambiguity_ratio=0.80)
        self.assertEqual(image.shape[:2], mask.shape)
        self.assertGreaterEqual(int(np.count_nonzero(mask)), 1900)
        self.assertEqual(0, int(np.count_nonzero(mask[:, :30])))

        image[10:30, 5:35, 0] = 230
        with self.assertRaises(perception.PerceptionError):
            perception.select_red_component(
                image, min_pixels=500, ambiguity_ratio=0.30)

    def test_red_component_selection_rejects_aubo_orange(self):
        perception = _load_perception()
        image = np.zeros((60, 80, 3), dtype=np.uint8)
        image[10:50, 20:60] = (200, 70, 0)
        with self.assertRaises(perception.PerceptionError):
            perception.select_red_component(image, min_pixels=100)

    def test_backprojection_uses_calibration_and_rejects_invalid_depth(self):
        perception = _load_perception()
        depth = np.full((3, 3), np.nan, dtype=np.float32)
        depth[1, 2] = 2.0
        mask = np.zeros((3, 3), dtype=np.uint8)
        mask[1, 2] = 255
        points = perception.backproject_mask(
            mask, depth, (2.0, 0.0, 1.0,
                          0.0, 2.0, 1.0,
                          0.0, 0.0, 1.0),
            min_depth=0.2, max_depth=4.0)
        np.testing.assert_allclose(points, [[1.0, 0.0, 2.0]])
        depth[1, 2] = 0.0
        with self.assertRaises(perception.PerceptionError):
            perception.backproject_mask(
                mask, depth, np.eye(3), min_depth=0.2, max_depth=4.0)

    def test_pose_estimation_fits_top_surface_and_pi_periodic_yaw(self):
        perception = _load_perception()
        yaw = 0.38
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        samples = []
        for along in np.linspace(-0.12, 0.12, 25):
            for across in np.linspace(-0.0265, 0.0265, 9):
                samples.append((
                    1.7 + cosine * along - sine * across,
                    -0.2 + sine * along + cosine * across,
                    0.115))
        pose = perception.estimate_target_pose(
            np.asarray(samples), target_height=0.115,
            top_surface_tolerance=0.01)
        self.assertAlmostEqual(1.7, pose[0], places=3)
        self.assertAlmostEqual(-0.2, pose[1], places=3)
        self.assertAlmostEqual(0.0575, pose[2], places=3)
        self.assertLess(perception.yaw_error_mod_pi(pose[3], yaw), 0.01)

    def test_pose_fusion_requires_consistent_finite_sensor_samples(self):
        perception = _load_perception()
        samples = (
            (2.00, 0.01, 0.057, 0.02),
            (2.01, 0.00, 0.058, 0.01),
            (1.99, -0.01, 0.0575, math.pi - 0.01),
        )
        fused = perception.fuse_pose_samples(
            samples, max_position_spread=0.04, max_yaw_spread=0.08)
        np.testing.assert_allclose(fused[:3], [2.0, 0.0, 0.0575], atol=1e-3)
        self.assertLess(perception.yaw_error_mod_pi(fused[3], 0.0), 0.02)
        inconsistent = samples + ((2.30, 0.0, 0.0575, 0.0),)
        with self.assertRaises(perception.PerceptionError):
            perception.fuse_pose_samples(
                inconsistent, max_position_spread=0.04,
                max_yaw_spread=0.08)

    def test_depth_registration_uses_real_optical_extrinsics(self):
        perception = _load_perception()
        depth = np.full((3, 3), np.nan, dtype=np.float32)
        depth[1, 1] = 2.0
        with warnings.catch_warnings():
            warnings.simplefilter("error", RuntimeWarning)
            registered = perception.register_depth_to_color(
                depth, np.eye(3), np.eye(3), (3, 3),
                rotation=np.eye(3), translation=np.array((1.0, 0.0, 0.0)))
        self.assertTrue(math.isnan(float(registered[1, 1])))
        self.assertAlmostEqual(2.0, float(registered[1, 2]))
        with self.assertRaises(perception.PerceptionError):
            perception.register_depth_to_color(
                depth, np.eye(3), np.eye(3), (3, 3),
                rotation=np.zeros((2, 2)), translation=np.zeros(3))

    def test_observation_timestamps_must_be_current_sim_time(self):
        perception = _load_perception()
        perception.validate_observation_stamps(
            (9.96, 9.98, 9.96, 9.98), now=10.0,
            max_age=0.5, max_future_skew=0.1)
        with self.assertRaises(perception.PerceptionError):
            perception.validate_observation_stamps(
                (8.0, 8.0, 8.0, 8.0), now=10.0,
                max_age=0.5, max_future_skew=0.1)
        with self.assertRaises(perception.PerceptionError):
            perception.validate_observation_stamps(
                (10.2, 10.2, 10.2, 10.2), now=10.0,
                max_age=0.5, max_future_skew=0.1)

    def test_observers_are_camera_configured_and_have_no_gt_input(self):
        source = OBSERVER.read_text(encoding="utf-8")
        for required in (
                "message_filters.ApproximateTimeSynchronizer",
                "select_red_component", "register_depth_to_color",
                "backproject_mask", "estimate_target_pose",
                "fuse_pose_samples", "lookup_transform",
                "validate_observation_stamps",
                "PoseStamped", "rospy.is_shutdown()",
                "def publish_pose",
                "color_frame, depth_frame, stamps =",
                "color.header.frame_id", "color_info.header.frame_id",
                "depth_info.header.frame_id", "to_sec()"):
            self.assertIn(required, source)
        for parameter in (
                "~color_topic", "~depth_topic", "~color_info_topic",
                "~depth_info_topic", "~output_topic", "~target_frame",
                "~camera_optical_frame"):
            self.assertIn(parameter, source)
        lowered = source.lower()
        for forbidden in (
                "/gazebo/model", "getmodelstate", "setmodelstate",
                "teleport", "attach", "benchmark", "provenance"):
            self.assertNotIn(forbidden, lowered)
        self.assertGreaterEqual(source.count("validate_observation_stamps("), 2)

    def test_air_and_ground_observer_configs_use_public_d435_topics(self):
        import yaml

        air = yaml.safe_load(AIR_CONFIG.read_text(encoding="utf-8"))
        ground = yaml.safe_load(GROUND_CONFIG.read_text(encoding="utf-8"))
        self.assertEqual(
            "/uav1/camera/color/image_raw", air["color_topic"])
        self.assertEqual(
            "/uav1/camera/depth/image_raw", air["depth_topic"])
        self.assertEqual(
            "uav1/camera_link",
            air["camera_optical_frame"])
        self.assertEqual("/air_observer/target_pose", air["output_topic"])
        self.assertEqual(
            "/ground/d435/color/image_raw", ground["color_topic"])
        self.assertEqual(
            "/ground/d435/depth/image_raw", ground["depth_topic"])
        self.assertEqual(
            "ground/d435_color_optical_frame",
            ground["camera_optical_frame"])
        self.assertEqual(
            "/ground_observer/target_pose", ground["output_topic"])
        for config in (air, ground):
            self.assertEqual("world", config["target_frame"])
            self.assertEqual(0.115, config["target_height"])
            self.assertGreaterEqual(config["stable_frames"], 3)
            self.assertEqual(1.0, config["max_observation_age"])
            self.assertGreaterEqual(config["max_future_skew"], 0.0)

    def test_observer_launch_starts_two_independent_sensor_nodes(self):
        root = ET.parse(str(OBSERVERS_LAUNCH)).getroot()
        nodes = root.findall("node")
        self.assertEqual(
            {"air_target_observer", "ground_target_observer"},
            {node.get("name") for node in nodes})
        for node in nodes:
            self.assertEqual("air_ground_pick_demo", node.get("pkg"))
            self.assertEqual("red_target_observer.py", node.get("type"))
            rosparam = node.find("rosparam")
            self.assertIsNotNone(rosparam)
            self.assertIn("_observer.yaml", rosparam.get("file"))

    def test_one_shot_flight_sequence_is_minimal_and_ordered(self):
        flight = _load_module(
            FLIGHT, "air_ground_pick_flight_test_target")
        sequence = flight.OneShotFlightSequence()
        transitions = (
            ("preflight_ready", "ARM", "ARMING"),
            ("armed", "ENTER_COMMAND_CONTROL", "COMMAND_CONTROL"),
            ("command_control_ready", "TAKEOFF", "TAKEOFF"),
            ("airborne", "MOVE_VIEW", "AIR_VIEW"),
            ("at_view", "OBSERVE", "AIR_OBSERVE"),
            ("fresh_observation", "HANDOFF", "AIR_HANDOFF"),
            ("handoff_published", "LAND", "LANDING"),
            ("landed", "COMPLETE", "COMPLETE"),
        )
        actions = []
        for event, expected_action, expected_phase in transitions:
            actions.append(sequence.advance(event))
            self.assertEqual(expected_action, actions[-1])
            self.assertEqual(expected_phase, sequence.phase)
        self.assertEqual(1, actions.count("TAKEOFF"))
        self.assertEqual(1, actions.count("MOVE_VIEW"))
        self.assertEqual(1, actions.count("LAND"))
        with self.assertRaises(flight.FlightError):
            sequence.advance("landed")

    def test_flight_health_arrival_observation_and_safe_stop(self):
        flight = _load_module(
            FLIGHT, "air_ground_pick_flight_health_test_target")
        self.assertTrue(flight.preflight_ready(
            connected=True, odom_valid=True, armed=False, failsafe=False,
            velocity=(0.01, 0.0, 0.0), state_age=0.1,
            control_age=0.1, max_age=1.0, max_speed=0.1))
        self.assertFalse(flight.preflight_ready(
            connected=True, odom_valid=True, armed=False, failsafe=False,
            velocity=(0.2, 0.0, 0.0), state_age=0.1,
            control_age=0.1, max_age=1.0, max_speed=0.1))
        self.assertTrue(flight.message_is_fresh(age=0.1, max_age=1.0))
        self.assertFalse(flight.message_is_fresh(age=1.1, max_age=1.0))
        self.assertTrue(flight.position_reached(
            (1.02, -0.01, 1.48), (1.0, 0.0, 1.5),
            (0.02, 0.0, 0.01), tolerance=0.05, max_speed=0.05))
        self.assertFalse(flight.position_reached(
            (1.02, -0.01, 1.48), (1.0, 0.0, 1.5),
            (0.20, 0.0, 0.0), tolerance=0.05, max_speed=0.05))
        self.assertTrue(flight.observation_is_fresh(
            stamp=10.8, now=11.0, max_age=0.5,
            frame_id="world", expected_frame="world"))
        self.assertFalse(flight.observation_is_fresh(
            stamp=10.0, now=11.0, max_age=0.5,
            frame_id="world", expected_frame="world"))
        self.assertEqual(
            ("HOLD", "LAND"),
            flight.safe_stop_actions(armed=True, command_control=True))
        self.assertEqual(
            ("AUTO_LAND",),
            flight.safe_stop_actions(armed=True, command_control=False))
        self.assertEqual(
            (), flight.safe_stop_actions(
                armed=False, command_control=False))

    def test_ground_target_transform_heading_standoff_and_velocity_bounds(self):
        approach = _load_module(
            APPROACH, "air_ground_pick_approach_test_target")
        target_x, target_y = approach.world_to_base(
            target_xy=(2.0, 0.0), base_xy_yaw=(3.5, 0.0, math.pi))
        self.assertAlmostEqual(1.5, target_x, places=6)
        self.assertAlmostEqual(0.0, target_y, places=6)
        limits = approach.ApproachLimits(
            standoff=0.82, distance_tolerance=0.05,
            heading_tolerance=0.10, turn_in_place_angle=0.45,
            linear_gain=0.8, angular_gain=1.5,
            max_linear=0.25, max_angular=0.50,
            obstacle_stop_distance=0.45)
        command = approach.compute_command(
            target_x=target_x, target_y=0.30,
            forward_clearance=2.0, limits=limits)
        self.assertGreater(command.linear_x, 0.0)
        self.assertGreater(command.angular_z, 0.0)
        self.assertLessEqual(abs(command.linear_x), limits.max_linear)
        self.assertLessEqual(abs(command.angular_z), limits.max_angular)
        self.assertFalse(command.reached)
        self.assertFalse(command.blocked)

        turn = approach.compute_command(
            target_x=0.20, target_y=1.0,
            forward_clearance=2.0, limits=limits)
        self.assertEqual(0.0, turn.linear_x)
        self.assertEqual(limits.max_angular, turn.angular_z)

        stopped = approach.compute_command(
            target_x=0.82, target_y=0.01,
            forward_clearance=2.0, limits=limits)
        self.assertEqual((0.0, 0.0),
                         (stopped.linear_x, stopped.angular_z))
        self.assertTrue(stopped.reached)

        no_motion = approach.compute_command(
            target_x=0.0, target_y=0.0,
            forward_clearance=2.0, limits=limits)
        self.assertEqual((0.0, 0.0),
                         (no_motion.linear_x, no_motion.angular_z))
        self.assertTrue(no_motion.reached)

    def test_ground_approach_scan_safety_and_freshness(self):
        approach = _load_module(
            APPROACH, "air_ground_pick_approach_safety_test_target")
        limits = approach.ApproachLimits(
            standoff=0.82, distance_tolerance=0.05,
            heading_tolerance=0.10, turn_in_place_angle=0.45,
            linear_gain=0.8, angular_gain=1.5,
            max_linear=0.25, max_angular=0.50,
            obstacle_stop_distance=0.45)
        ranges = [float("inf")] * 181
        ranges[90] = 0.40
        clearance = approach.forward_clearance(
            ranges, angle_min=-math.pi / 2.0,
            angle_increment=math.pi / 180.0,
            sector_half_angle=0.30, range_min=0.05, range_max=10.0)
        self.assertAlmostEqual(0.40, clearance)
        blocked = approach.compute_command(
            target_x=1.5, target_y=0.0,
            forward_clearance=clearance, limits=limits)
        self.assertEqual((0.0, 0.0),
                         (blocked.linear_x, blocked.angular_z))
        self.assertTrue(blocked.blocked)
        self.assertFalse(blocked.reached)
        with self.assertRaises(approach.ApproachError):
            approach.forward_clearance(
                [float("nan")] * 10, angle_min=-0.2,
                angle_increment=0.04, sector_half_angle=0.3,
                range_min=0.05, range_max=10.0)
        self.assertTrue(approach.scan_is_fresh(
            stamp=20.8, now=21.0, max_age=0.5))
        self.assertFalse(approach.scan_is_fresh(
            stamp=20.0, now=21.0, max_age=0.5))
        self.assertTrue(approach.transform_is_fresh(
            stamp=20.8, now=21.0, max_age=0.5))
        self.assertFalse(approach.transform_is_fresh(
            stamp=20.0, now=21.0, max_age=0.5))
        self.assertAlmostEqual(
            0.75, approach.travel_distance(
                initial_xy=(3.5, 0.0), current_xy=(2.75, 0.0)))
        self.assertIsNone(approach.clearance_for_status(float("inf")))
        self.assertEqual(0.40, approach.clearance_for_status(0.40))

    def test_orchestrator_uses_runtime_interfaces_without_gt_or_teleport(self):
        import yaml

        config = yaml.safe_load(DEMO_CONFIG.read_text(encoding="utf-8"))
        self.assertEqual(
            "/uav1/prometheus/state", config["uav_state_topic"])
        self.assertEqual(
            "/uav1/prometheus/control_state",
            config["uav_control_state_topic"])
        self.assertEqual(
            "/uav1/prometheus/setup", config["uav_setup_topic"])
        self.assertEqual(
            "/uav1/prometheus/command", config["uav_command_topic"])
        self.assertEqual(
            "/air_observer/target_pose", config["air_pose_topic"])
        self.assertEqual("/ground/scan", config["ground_scan_topic"])
        self.assertEqual("/ground/cmd_vel", config["ground_cmd_topic"])
        self.assertEqual("world", config["world_frame"])
        self.assertEqual("ground/base_link", config["ground_base_frame"])
        self.assertGreater(config["ground_standoff"],
                           config["obstacle_stop_distance"])
        self.assertLessEqual(config["max_ground_linear"], 0.25)
        self.assertLessEqual(config["max_ground_angular"], 0.50)
        self.assertGreater(config["max_ground_travel"], 0.0)
        self.assertGreater(config["ground_timeout"], 0.0)
        self.assertGreater(config["ground_tf_max_age"], 0.0)

        source = ORCHESTRATOR.read_text(encoding="utf-8")
        for required in (
                "OneShotFlightSequence", "preflight_ready",
                "observation_is_fresh", "world_to_base",
                "forward_clearance", "compute_command",
                "transform_is_fresh",
                "UAVSetup", "UAVCommand", "UAVControlState", "UAVState",
                "LaserScan", "Twist", "PoseStamped", "lookup_transform",
                "AIR_HANDOFF", "GROUND_STOPPED", "safe_stop_actions"):
            self.assertIn(required, source)
        lowered = source.lower()
        for forbidden in (
                "/gazebo/model", "getmodelstate", "setmodelstate",
                "teleport", "attach", "benchmark", "provenance",
                "task-aware", "rm4d"):
            self.assertNotIn(forbidden, lowered)
        safe_land = source.split("def _safe_land", 1)[1].split(
            "def run", 1)[0]
        self.assertIn("while not rospy.is_shutdown()", safe_land)
        self.assertIn("self._landing_timeout", safe_land)
        self.assertIn("state.armed", safe_land)
        self.assertIn(
            "self._uav_state_is_current(snapshot, now)", safe_land)
        self.assertEqual(1, safe_land.count("self._snapshot()"))
        repeat_until = source.split("def _repeat_until", 1)[1].split(
            "def _preflight_is_ready", 1)[0]
        self.assertIn("health_check", repeat_until)
        self.assertIn("flight health lost", repeat_until)
        observe = source.split("def _observe_and_handoff", 1)[1].split(
            "def _land", 1)[0]
        self.assertIn(
            "self._command_flight_is_healthy(\n"
            "                    snapshot, time.monotonic())", observe)
        landing = source.split("def _land", 1)[1].split(
            "def _yaw_from_quaternion", 1)[0]
        self.assertIn(
            "self._uav_state_is_current(snapshot, now)", landing)
        self.assertEqual(1, landing.count("self._snapshot()"))
        flight_health = source.split(
            "def _flight_state_is_fresh", 1)[1].split(
                "def _arm", 1)[0]
        self.assertIn("snapshot=None", flight_health)
        self.assertIn(
            "self._uav_state_is_current(snapshot, now)", flight_health)
        self.assertIn(
            "self._control_state_is_current(snapshot, now)", flight_health)

    def test_orchestrator_direct_execution_does_not_shadow_package(self):
        system_python = Path("/usr/bin/python3")
        if not system_python.is_file():
            self.skipTest("system Python is unavailable")
        python_paths = (
            str(PACKAGE / "src"),
            str(ROOT / "install/p450-clean/lib/python3/dist-packages"),
            "/opt/ros/noetic/lib/python3/dist-packages",
        )
        environment = os.environ.copy()
        environment["PYTHONPATH"] = os.pathsep.join(python_paths)
        result = subprocess.run(
            [str(system_python), "-B", str(ORCHESTRATOR),
             "--check-imports"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, timeout=10.0, env=environment)
        self.assertEqual(0, result.returncode, result.stderr)


if __name__ == "__main__":
    unittest.main()
