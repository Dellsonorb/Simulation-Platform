#!/usr/bin/env python3

import pathlib
import unittest

import yaml
from catkin_pkg.package import parse_package


PACKAGE = pathlib.Path(__file__).resolve().parents[1]


class RosContractTest(unittest.TestCase):
    def test_package_manifest_is_valid(self):
        package = parse_package(str(PACKAGE / "package.xml"))
        package.validate()

    def test_catkin_components_are_declared_for_build_and_runtime(self):
        package = parse_package(str(PACKAGE / "package.xml"))
        build_names = {dependency.name for dependency in package.build_depends}
        exec_names = {dependency.name for dependency in package.exec_depends}
        expected = {
            "cv_bridge", "gazebo_msgs", "geometry_msgs", "mavros_msgs",
            "message_filters", "nav_msgs", "rospy", "sensor_msgs", "std_msgs",
            "std_srvs", "tf", "tf2_ros", "visualization_msgs",
        }
        self.assertTrue(expected.issubset(build_names))
        self.assertTrue(expected.issubset(exec_names))

    def test_perception_node_contract(self):
        source = (PACKAGE / "scripts" / "aerial_brick_pose_node.py").read_text()
        for token in (
            "ApproximateTimeSynchronizer",
            "camera/color/image_raw",
            "camera/depth/image_raw",
            "camera/color/camera_info",
            "camera/depth/camera_info",
            "aligned_depth_to_color/image_raw",
            "brick_mask",
            "brick_debug_rgb",
            "brick_points",
            "brick_pose",
            "PoseStamped",
            "quality_status",
            "world",
            "rospy.ROSException",
        ):
            self.assertIn(token, source)
        self.assertNotIn("/brick_pose", source)
        self.assertNotIn("bunker", source.lower())

    def test_valid_pose_and_quality_share_exact_measurement_identity(self):
        source = (PACKAGE / "scripts" / "aerial_brick_pose_node.py").read_text()
        for token in (
            "measurement_stamp_secs",
            "measurement_stamp_nsecs",
            "source_seq",
            "color_message.header.seq",
        ):
            self.assertIn(token, source)
        self.assertIn("published_seq = self._publish_pose_and_marker", source)
        self.assertIn("source_seq=int(published_seq)", source)
        self.assertIn("source_image_seq=int(color_message.header.seq)", source)

    def test_world_tf_bridge_uses_ground_truth_and_receive_time(self):
        source = (PACKAGE / "scripts" / "world_tf_bridge.py").read_text()
        self.assertIn("prometheus/ground_truth", source)
        self.assertIn("rospy.Time.now()", source)
        self.assertIn("camera_color_optical_frame", source)
        self.assertIn("camera_depth_optical_frame", source)

    def test_configuration_declares_alignment_and_quality_limits(self):
        config = (PACKAGE / "config" / "perception.yaml").read_text()
        for token in (
            "sync_slop",
            "min_depth",
            "max_depth",
            "min_points",
            "stable_frames",
            "max_position_spread",
            "max_yaw_spread",
            "brick_height",
        ):
            self.assertIn(token, config)
        parsed = yaml.safe_load(config)
        self.assertGreaterEqual(parsed["min_points"], 150)

    def test_final_baseline_uses_shape_component_mask_frontend(self):
        parsed = yaml.safe_load(
            (PACKAGE / "config" / "perception.yaml").read_text())
        self.assertEqual("rgb_shape_component", parsed["mask_frontend"])
        for key in (
                "min_component_area", "max_component_area",
                "min_aspect_ratio", "max_aspect_ratio",
                "target_aspect_ratio", "side_up_target_aspect_ratio",
                "min_rectangularity",
                "boundary_margin", "min_frontend_score",
                "frontend_ambiguity_margin"):
            self.assertIn(key, parsed)
        self.assertEqual(2.0, parsed["side_up_target_aspect_ratio"])
        self.assertEqual(0.60, parsed["min_frontend_score"])

    def test_node_keeps_backend_and_measurement_key_behind_frontend(self):
        source = (PACKAGE / "scripts" / "aerial_brick_pose_node.py").read_text()
        self.assertIn("make_mask_frontend", source)
        self.assertIn("projected_target_aspect_ratio", source)
        self.assertIn("mask_result = self.mask_frontend.segment(rgb)", source)
        self.assertIn("mask = mask_result.mask", source)
        self.assertIn('"mask_frontend": self.mask_frontend_name', source)
        self.assertIn('"frontend": mask_result.audit', source)
        self.assertLess(source.index("mask_result = self.mask_frontend.segment(rgb)"),
                        source.index("backproject_mask("))
        self.assertIn("register_depth_to_color(", source)
        self.assertIn("estimate_brick_pose(", source)
        self.assertNotIn("task_aware_approach", source)
        self.assertNotIn("aerial_traversability", source)

    def test_viewpoint_adapter_reuses_prometheus_control(self):
        source = (PACKAGE / "scripts" / "aerial_viewpoint_mission.py").read_text()
        for token in (
            "UAVSetup",
            "UAVCommand",
            "UAVState",
            "UAVControlState",
            "SAMPLING:",
            "Current_Pos_Hover",
            "Land",
            "waypoint_timeout",
        ):
            self.assertIn(token, source)
        self.assertNotIn("set_model_state", source)

    def test_launch_reuses_official_p450_and_is_ground_independent(self):
        launch = (PACKAGE / "launch" / "m1_aerial_perception.launch").read_text()
        world = (PACKAGE / "worlds" / "m1_brick.world").read_text()
        self.assertIn("sitl_p450_d435i.launch", launch)
        self.assertIn("uav_control_main_outdoor.launch", launch)
        self.assertIn("m1_brick.world", launch)
        self.assertIn("m1_brick", world)
        self.assertIn("0.20 0.10 0.053", world)
        combined = (launch + world).lower()
        self.assertNotIn("bunker", combined)
        self.assertNotIn("aubo", combined)

    def test_rviz_shows_rgb_aligned_depth_mask_cloud_and_pose(self):
        rviz = (PACKAGE / "rviz" / "m1_aerial_perception.rviz").read_text()
        for topic in (
            "/uav1/camera/color/image_raw",
            "/m1_aerial_brick_pose/aligned_depth_to_color/image_raw",
            "/m1_aerial_brick_pose/brick_mask",
            "/m1_aerial_brick_pose/brick_points",
            "/m1_aerial_brick_pose/brick_pose_marker",
        ):
            self.assertIn(topic, rviz)

    def test_validation_brick_is_static_so_rotor_wash_cannot_move_ground_truth(self):
        world = (PACKAGE / "worlds" / "m1_brick.world").read_text()
        self.assertIn("<model name=\"m1_brick\">", world)
        self.assertIn("<static>true</static>", world)


if __name__ == "__main__":
    unittest.main()
