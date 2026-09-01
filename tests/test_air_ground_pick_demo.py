#!/usr/bin/env python3

import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import threading
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
GRASP = PACKAGE / "src/air_ground_pick_demo/grasp.py"
TARGET_MODEL = PACKAGE / "models/pick_target/model.sdf"
TARGET_CONFIG = PACKAGE / "models/pick_target/model.config"
OBSERVER = PACKAGE / "scripts/red_target_observer.py"
AIR_CONFIG = PACKAGE / "config/air_observer.yaml"
GROUND_CONFIG = PACKAGE / "config/ground_observer.yaml"
OBSERVERS_LAUNCH = PACKAGE / "launch/target_observers.launch"
ORCHESTRATOR = PACKAGE / "scripts/run_air_ground_pick_demo.py"
TARGET_SPAWNER = PACKAGE / "scripts/spawn_pick_target.py"
DEMO_CONFIG = PACKAGE / "config/demo.yaml"
DEMO_LAUNCH = PACKAGE / "launch/air_ground_pick_demo.launch"
DEMO_CHECKER = ROOT / "scripts/check_air_ground_pick_demo.py"
DEMO_SMOKE = ROOT / "scripts/smoke_air_ground_pick_demo.bash"


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
        self.assertTrue(ORCHESTRATOR.stat().st_mode & 0o111)
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
                "runtime_ready_topic", "wait_for_message", "Bool",
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
            parameters = {item.get("name"): item.get("value")
                          for item in node.findall("param")}
            self.assertEqual(
                "/ground/runtime_ready", parameters["runtime_ready_topic"])

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

    def test_ag95_feasibility_uses_real_opening_and_measured_joint(self):
        grasp = _load_module(
            GRASP, "air_ground_pick_grasp_feasibility_test_target")
        feasibility = grasp.check_target_feasibility(
            target_size=(0.240, 0.053, 0.115),
            maximum_opening=0.0952, opening_margin=0.002)
        self.assertTrue(feasibility.feasible)
        self.assertAlmostEqual(0.053, feasibility.grasp_span)
        self.assertAlmostEqual(0.055, feasibility.required_opening)
        self.assertAlmostEqual(
            0.0952,
            grasp.conservative_jaw_opening(
                master_joint_position=0.0,
                maximum_opening=0.0952,
                maximum_joint_position=0.93))
        self.assertGreater(
            grasp.conservative_jaw_opening(
                master_joint_position=0.20,
                maximum_opening=0.0952,
                maximum_joint_position=0.93),
            feasibility.required_opening)
        self.assertFalse(grasp.check_target_feasibility(
            target_size=(0.240, 0.100, 0.115),
            maximum_opening=0.0952, opening_margin=0.002).feasible)
        with self.assertRaises(grasp.GraspError):
            grasp.conservative_jaw_opening(
                master_joint_position=-0.01,
                maximum_opening=0.0952,
                maximum_joint_position=0.93)

    def test_top_down_grasp_uses_tcp_geometry_and_vertical_cartesian_moves(self):
        grasp = _load_module(
            GRASP, "air_ground_pick_grasp_geometry_test_target")
        poses = grasp.generate_top_down_grasp(
            target=(2.0, 0.0, 0.0575, 0.0),
            target_size=(0.240, 0.053, 0.115),
            pregrasp_height=0.15, lift_height=0.15,
            finger_pad_lower_edge_offset=0.0156,
            contact_overlap=0.020, surface_clearance=0.010)
        self.assertEqual((2.0, 0.0), poses.grasp.position[:2])
        self.assertAlmostEqual(0.0794, poses.grasp.position[2])
        self.assertAlmostEqual(0.2294, poses.pregrasp.position[2])
        self.assertAlmostEqual(0.2294, poses.lift.position[2])
        self.assertEqual(poses.grasp.orientation, poses.pregrasp.orientation)
        self.assertEqual(poses.grasp.orientation, poses.lift.orientation)

        rotation = grasp.quaternion_matrix(poses.grasp.orientation)
        np.testing.assert_allclose(
            rotation.dot(np.array((1.0, 0.0, 0.0))),
            (0.0, 0.0, -1.0), atol=1e-9)
        np.testing.assert_allclose(
            rotation.dot(np.array((0.0, 1.0, 0.0))),
            (0.0, 1.0, 0.0), atol=1e-9)

        rotated = grasp.generate_top_down_grasp(
            target=(2.0, 0.0, 0.0575, 0.4),
            target_size=(0.240, 0.053, 0.115),
            pregrasp_height=0.15, lift_height=0.15,
            finger_pad_lower_edge_offset=0.0156,
            contact_overlap=0.020, surface_clearance=0.010)
        rotated_matrix = grasp.quaternion_matrix(rotated.grasp.orientation)
        np.testing.assert_allclose(
            rotated_matrix.dot(np.array((0.0, 1.0, 0.0))),
            (-math.sin(0.4), math.cos(0.4), 0.0), atol=1e-9)

        with self.assertRaises(grasp.GraspError):
            grasp.generate_top_down_grasp(
                target=(2.0, 0.0, 0.0575, 0.0),
                target_size=(0.240, 0.053, 0.115),
                pregrasp_height=0.15, lift_height=0.15,
                finger_pad_lower_edge_offset=0.0156,
                contact_overlap=0.090, surface_clearance=0.010)

    def test_contact_classification_requires_target_on_both_real_pads(self):
        grasp = _load_module(
            GRASP, "air_ground_pick_grasp_contact_test_target")
        target = "pick_target::pick_target_link::pick_target_collision"
        left = "ground_robot::ground/left_finger_pad::collision"
        right = "ground_robot::ground/right_finger_pad::collision"
        ground = "ground_plane::link::collision"
        self.assertEqual(
            (True, True), grasp.contact_sides(
                ((target, left), (right, target), (target, ground))))
        self.assertEqual(
            (True, False), grasp.contact_sides(((left, target),)))
        self.assertEqual(
            (False, False), grasp.contact_sides(
                ((left, right), (target, ground))))

    def test_cartesian_trajectory_ends_at_rest(self):
        grasp = _load_module(
            GRASP, "air_ground_pick_grasp_stop_test_target")
        terminal = type("Point", (), {})()
        terminal.velocities = [0.2, -0.1]
        terminal.accelerations = [0.4, -0.3]
        joint_trajectory = type("JointTrajectory", (), {})()
        joint_trajectory.joint_names = ["joint_a", "joint_b"]
        joint_trajectory.points = [terminal]
        trajectory = type("Trajectory", (), {})()
        trajectory.joint_trajectory = joint_trajectory

        self.assertIs(trajectory, grasp.zero_terminal_motion(trajectory))
        self.assertEqual([0.0, 0.0], terminal.velocities)
        self.assertEqual([0.0, 0.0], terminal.accelerations)

    def test_pick_runtime_config_names_real_arm_gripper_and_sensor_interfaces(self):
        import yaml

        config = yaml.safe_load(DEMO_CONFIG.read_text(encoding="utf-8"))
        self.assertEqual(
            "/ground/arm_controller/follow_joint_trajectory",
            config["arm_action"])
        self.assertEqual(
            "/ground/gripper_controller/follow_joint_trajectory",
            config["gripper_action"])
        self.assertEqual("/ground/joint_states", config["joint_state_topic"])
        self.assertEqual(
            "/ground_observer/target_pose", config["ground_pose_topic"])
        self.assertEqual("/pick_target/contacts", config["contact_topic"])
        self.assertEqual(
            ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"],
            config["observation_joint_names"])
        self.assertEqual(6, len(config["observation_joint_positions"]))
        self.assertEqual([0.240, 0.053, 0.115], config["target_size"])
        self.assertEqual(0.0952, config["maximum_gripper_opening"])
        self.assertEqual(0.0, config["gripper_open_position"])
        self.assertGreater(config["gripper_closed_position"], 0.0)
        self.assertGreaterEqual(config["minimum_gripper_closed_joint"], 0.20)
        self.assertGreaterEqual(config["minimum_lift"], 0.10)
        self.assertAlmostEqual(0.005, config["planning_position_tolerance"])
        self.assertAlmostEqual(
            0.02, config["planning_orientation_tolerance"])
        self.assertAlmostEqual(0.02, config["pose_position_tolerance"])
        self.assertAlmostEqual(0.12, config["pose_orientation_tolerance"])
        self.assertLessEqual(config["cartesian_velocity_scaling"], 0.05)
        self.assertGreater(config["pose_settle_timeout"], 0.0)
        self.assertLessEqual(config["pose_settle_timeout"], 5.0)

    def test_demo_launch_composes_platform_moveit_target_and_sensor_nodes(self):
        root = ET.parse(str(DEMO_LAUNCH)).getroot()
        arguments = {item.get("name"): item.get("default")
                     for item in root.findall("arg")}
        self.assertEqual("true", arguments["run_demo"])
        self.assertEqual("true", arguments["start_moveit"])
        self.assertIn("px4_workdir", arguments)
        includes = [item.get("file") for item in root.findall("include")]
        self.assertTrue(any("air_ground_standalone.launch" in item
                            for item in includes))
        shared = next(item for item in root.findall("include")
                      if "air_ground_standalone.launch" in item.get("file"))
        shared_args = {item.get("name"): item.get("value")
                       for item in shared.findall("arg")}
        self.assertEqual("$(arg px4_workdir)", shared_args["px4_workdir"])
        moveit = next(item for item in root.findall("include")
                      if "ground_move_group.launch" in item.get("file"))
        self.assertEqual("$(arg start_moveit)", moveit.get("if"))
        self.assertTrue(any("target_observers.launch" in item
                            for item in includes))
        nodes = root.findall("node")
        spawn = next(node for node in nodes
                     if node.get("name") == "spawn_pick_target")
        self.assertEqual("air_ground_pick_demo", spawn.get("pkg"))
        self.assertEqual("spawn_pick_target.py", spawn.get("type"))
        parameters = {item.get("name"): item.get("value")
                      for item in spawn.findall("param")}
        self.assertIn("pick_target/model.sdf", parameters["model_path"])
        self.assertEqual(
            "/uav1/camera/color/image_raw", parameters["air_image_topic"])
        self.assertEqual(
            "/ground/d435/color/image_raw",
            parameters["ground_image_topic"])
        orchestrator = next(node for node in nodes
                            if node.get("name") == "air_ground_pick_demo")
        self.assertEqual("air_ground_pick_demo", orchestrator.get("pkg"))
        self.assertEqual(
            "run_air_ground_pick_demo.py", orchestrator.get("type"))
        self.assertFalse(
            (PACKAGE / "scripts/air_ground_pick_demo.py").exists())
        self.assertEqual("$(arg run_demo)", orchestrator.get("if"))
        self.assertIn("demo.yaml", orchestrator.find("rosparam").get("file"))
        launch_text = DEMO_LAUNCH.read_text(encoding="utf-8").lower()
        self.assertNotIn("brick_pick", launch_text)
        self.assertNotIn("attachment", launch_text)

    def test_target_spawn_waits_for_robot_sensors_before_inserting_model(self):
        source = TARGET_SPAWNER.read_text(encoding="utf-8")
        for required in (
                "runtime_ready_topic", "wait_for_message", "Bool",
                "SpawnModel", "wait_for_service", "model_xml",
                "initial_pose", "air_image_topic", "ground_image_topic",
                "Image"):
            self.assertIn(required, source)
        lowered = source.lower()
        for forbidden in (
                "/gazebo/model_states", "get_model_state", "set_model_state",
                "teleport", "attach"):
            self.assertNotIn(forbidden, lowered)
        cmake = CMAKE.read_text(encoding="utf-8")
        self.assertIn("scripts/spawn_pick_target.py", cmake)

    def test_orchestrator_pick_phase_is_sensor_and_controller_driven(self):
        source = ORCHESTRATOR.read_text(encoding="utf-8")
        for required in (
                "MoveGroupCommander", "FollowJointTrajectoryAction",
                "FollowJointTrajectoryGoal", "JointTrajectoryPoint",
                "ContactsState", "RobotState", "contact_sides",
                "generate_top_down_grasp",
                "GROUND_OBSERVE", "GROUND_REFINED", "PREGRASP", "GRASP",
                "LIFT", "compute_cartesian_path", "set_pose_target",
                "bilateral"):
            self.assertIn(required, source)
        self.assertIn("zero_terminal_motion", source)
        self.assertIn("max_joint_speed", source)
        self.assertIn("measured=%.4f expected=%.4f", source)
        verify = source[source.index("    def _verify_tcp_pose("):
                        source.index("    def _execute_pregrasp(")]
        self.assertIn("lookup_transform", verify)
        self.assertIn("self._end_effector_link", verify)
        self.assertIn("self._pose_settle_timeout", verify)
        self.assertIn("self._wait_step()", verify)
        self.assertNotIn("get_current_pose", verify)
        initialize_moveit = source[
            source.index("    def _initialize_moveit("):
            source.index("    @staticmethod\n    def _quaternion_error")]
        self.assertIn(
            "self._planning_position_tolerance", initialize_moveit)
        self.assertIn(
            "self._planning_orientation_tolerance", initialize_moveit)
        cartesian = source[source.index("    def _execute_cartesian("):
                           source.index("    def _bilateral_contact_current(")]
        self.assertIn("_robot_state_from_joint_feedback()", cartesian)
        self.assertNotIn("get_current_state", cartesian)
        lowered = source.lower()
        for forbidden in (
                "/gazebo/model", "getmodelstate", "setmodelstate",
                "teleport", "attach", "brick_pick"):
            self.assertNotIn(forbidden, lowered)

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

    @staticmethod
    def _valid_demo_status_events():
        states = (
            "PREFLIGHT", "ARMING", "COMMAND_CONTROL", "TAKEOFF",
            "AIR_VIEW", "AIR_OBSERVE", "AIR_HANDOFF", "LANDING",
            "GROUND_APPROACH", "GROUND_STOPPED", "GROUND_OBSERVE",
            "GROUND_REFINED", "PREGRASP", "GRASP", "LIFTING", "LIFT",
        )
        events = [
            {"state": state, "ros_time": float(index + 1)}
            for index, state in enumerate(states)
        ]
        events[6]["observation_stamp"] = 6.9
        events[9].update({
            "ground_travel": 0.47,
            "target_distance": 0.82,
        })
        events[11]["observation_stamp"] = 11.9
        events[15].update({
            "tcp_lift": 0.14,
            "bilateral_contact": True,
        })
        return events

    def test_demo_checker_requires_exact_status_sequence_and_bounds(self):
        checker = _load_module(
            DEMO_CHECKER, "air_ground_pick_demo_checker_test_target")
        events = self._valid_demo_status_events()
        summary = checker.status_sequence_summary(
            events, maximum_ground_travel=1.10, minimum_tcp_lift=0.10)
        self.assertEqual(list(checker.EXPECTED_STATES), summary["states"])
        self.assertEqual(1, summary["takeoff_count"])
        self.assertEqual(1, summary["landing_count"])
        self.assertAlmostEqual(0.47, summary["ground_travel_m"])
        self.assertAlmostEqual(0.14, summary["tcp_lift_m"])

        out_of_order = list(events)
        out_of_order[7], out_of_order[8] = (
            out_of_order[8], out_of_order[7])
        with self.assertRaises(checker.DemoCheckError):
            checker.status_sequence_summary(
                out_of_order, maximum_ground_travel=1.10,
                minimum_tcp_lift=0.10)

        excessive_travel = [dict(event) for event in events]
        excessive_travel[9]["ground_travel"] = 1.11
        with self.assertRaises(checker.DemoCheckError):
            checker.status_sequence_summary(
                excessive_travel, maximum_ground_travel=1.10,
                minimum_tcp_lift=0.10)

        monitor = checker.DemoMonitor(
            rospy=None, goal_succeeded=3,
            contact_sides=lambda _pairs: (False, False),
            target_model="pick_target")
        message = type("Status", (), {})()
        message.data = json.dumps(events[0])
        monitor.status(message)
        message.data = json.dumps(events[3])
        callback = threading.Thread(target=monitor.status, args=(message,))
        callback.daemon = True
        callback.start()
        callback.join(0.2)
        self.assertFalse(callback.is_alive(), "status failure callback deadlocked")
        self.assertTrue(monitor.terminal.is_set())
        self.assertIn("unexpected status", monitor.snapshot()["error"])

    def test_demo_checker_requires_fresh_topic_observations(self):
        checker = _load_module(
            DEMO_CHECKER, "air_ground_pick_observation_checker_test_target")
        samples = (
            {"stamp": 6.5, "frame": "world"},
            {"stamp": 6.9, "frame": "world"},
        )
        summary = checker.observation_summary(
            samples, status_time=7.0, expected_stamp=6.9,
            expected_frame="world", maximum_age=1.0)
        self.assertAlmostEqual(0.1, summary["age_s"])
        self.assertAlmostEqual(6.9, summary["stamp"])
        with self.assertRaises(checker.DemoCheckError):
            checker.observation_summary(
                samples, status_time=8.0, expected_stamp=6.9,
                expected_frame="world", maximum_age=1.0)
        with self.assertRaises(checker.DemoCheckError):
            checker.observation_summary(
                ({"stamp": 6.9, "frame": "camera"},),
                status_time=7.0, expected_stamp=6.9,
                expected_frame="world", maximum_age=1.0)

    def test_demo_checker_requires_one_flight_controller_contact_and_lift(self):
        checker = _load_module(
            DEMO_CHECKER, "air_ground_pick_physics_checker_test_target")
        self.assertEqual(
            {"armed_seen": True, "landed_after_arm": True},
            checker.flight_cycle_summary((False, True, True, False)))
        with self.assertRaises(checker.DemoCheckError):
            checker.flight_cycle_summary((False, False))

        controller = checker.controller_success_summary(
            ((11.8, "observe"), (12.2, "pregrasp"),
             (13.0, "approach"), (15.0, "lift")),
            after_stamp=12.0, minimum_successes=3)
        self.assertEqual(3, controller["success_count"])
        with self.assertRaises(checker.DemoCheckError):
            checker.controller_success_summary(
                ((12.2, "pregrasp"), (13.0, "approach")),
                after_stamp=12.0, minimum_successes=3)

        self.assertEqual(
            {"bilateral_contact": True},
            checker.bilateral_contact_summary(True))
        with self.assertRaises(checker.DemoCheckError):
            checker.bilateral_contact_summary(False)

        lift = checker.target_lift_summary(
            baseline_z=0.0575, final_z=0.1775, minimum_lift=0.10)
        self.assertAlmostEqual(0.12, lift["target_lift_m"])
        with self.assertRaises(checker.DemoCheckError):
            checker.target_lift_summary(
                baseline_z=0.0575, final_z=0.1474, minimum_lift=0.10)

        finalized = checker.finalized_summary({
            "status": "CHECKS_PASS", "sequence": {"states": ["LIFT"]}})
        self.assertEqual("PASS", finalized["status"])
        with self.assertRaises(checker.DemoCheckError):
            checker.finalized_summary({"status": "PASS"})

    def test_demo_smoke_is_bounded_isolated_and_platform_only(self):
        self.assertTrue(DEMO_CHECKER.is_file())
        self.assertTrue(DEMO_SMOKE.is_file())
        self.assertTrue(DEMO_CHECKER.stat().st_mode & 0o111)
        self.assertTrue(DEMO_SMOKE.stat().st_mode & 0o111)
        help_result = subprocess.run(
            [str(DEMO_SMOKE), "--help"], stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, text=True, timeout=5.0)
        self.assertEqual(0, help_result.returncode, help_result.stderr)
        source = DEMO_SMOKE.read_text(encoding="utf-8")
        for required in (
                "with_p450_env.bash", "air_ground_pick_demo.launch",
                "check_air_ground_pick_demo.py", "ROS_MASTER_URI",
                "GAZEBO_MASTER_URI", "/usr/bin/timeout", "setsid roslaunch",
                "P450_GAZEBO_DISPLAY", "P450_GAZEBO_XAUTHORITY",
                "summary.json", "launch_pgid",
                '/bin/kill -INT -- "-$launch_pgid"',
                '/bin/kill -TERM -- "-$launch_pgid"',
                '/bin/kill -KILL -- "-$launch_pgid"',
                "--finalize-summary"):
            self.assertIn(required, source)
        lowered = source.lower()
        for forbidden in (
                "benchmark", "provenance", "artifact", "formal", "pilot",
                "lifecycle"):
            self.assertNotIn(forbidden, lowered)


if __name__ == "__main__":
    unittest.main()
