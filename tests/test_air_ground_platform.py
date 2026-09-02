#!/usr/bin/env python3

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src/platform/sim_platform_bringup"
LAUNCH = PACKAGE / "launch/air_ground_standalone.launch"
WORLD = PACKAGE / "worlds/air_ground_v1.world"
PACKAGE_XML = PACKAGE / "package.xml"
CHECKER = ROOT / "scripts/check_air_ground_runtime.py"
SMOKE = ROOT / "scripts/smoke_air_ground_standalone.bash"


def _load_checker():
    spec = importlib.util.spec_from_file_location(
        "check_air_ground_runtime", str(CHECKER))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class AirGroundPlatformTest(unittest.TestCase):
    def test_shared_launch_owns_one_gazebo_and_two_worldless_runtimes(self):
        root = ET.parse(str(LAUNCH)).getroot()
        includes = root.findall("include")
        files = [include.get("file") for include in includes]
        self.assertEqual(1, files.count(
            "$(find gazebo_ros)/launch/empty_world.launch"))
        self.assertIn(
            "$(find sim_platform_bringup)/launch/p450_runtime.launch", files)
        self.assertIn(
            "$(find ground_manipulator_runtime)/launch/ground_robot_runtime.launch",
            files)
        self.assertNotIn(
            "$(find bunker_sim_runtime)/launch/bunker_runtime.launch", files)

        ground = next(include for include in includes
                      if "ground_robot_runtime.launch" in include.get("file"))
        ground_args = {arg.get("name"): arg.get("value")
                       for arg in ground.findall("arg")}
        self.assertEqual("$(arg bunker_x)", ground_args["x"])
        self.assertEqual("$(arg bunker_y)", ground_args["y"])
        self.assertEqual("$(arg bunker_z)", ground_args["z"])
        self.assertEqual("$(arg bunker_yaw)", ground_args["yaw"])

        for runtime in (
                PACKAGE / "launch/p450_runtime.launch",
                ROOT / (
                    "src/platform/ground_manipulator_runtime/launch/"
                    "ground_robot_runtime.launch")):
            runtime_root = ET.parse(str(runtime)).getroot()
            self.assertFalse(any("empty_world.launch" in include.get("file", "")
                                 for include in runtime_root.findall(".//include")))
            self.assertFalse(any(
                node.get("pkg") == "tf2_ros"
                and (" world " in " %s " % node.get("args", "")
                     or " map " in " %s " % node.get("args", ""))
                for node in runtime_root.findall(".//node")))

        localization = {
            node.get("name"): node.get("args")
            for node in root.findall("./node")
            if node.get("pkg") == "tf2_ros"
        }
        self.assertEqual({
            "sim_world_to_map": "0 0 0 0 0 0 1 world map",
            "sim_localization_uav1": (
                "$(arg uav1_init_x) $(arg uav1_init_y) "
                "$(arg uav1_init_z) $(arg uav1_init_yaw) 0 0 "
                "map uav1/odom"),
            "sim_localization_ground": (
                "$(arg bunker_x) $(arg bunker_y) $(arg bunker_z) "
                "$(arg bunker_yaw) 0 0 map ground/odom"),
        }, localization)

    def test_shared_world_provides_ground_and_a_lidar_landmark(self):
        root = ET.parse(str(WORLD)).getroot()
        world = root.find("world")
        self.assertIsNotNone(world)
        uris = [include.findtext("uri") for include in world.findall("include")]
        self.assertIn("model://ground_plane", uris)
        obstacle = world.find("./model[@name='ground_scan_obstacle']")
        self.assertIsNotNone(obstacle)
        self.assertEqual("5.0 0.0 0.5 0 0 0", obstacle.findtext("pose"))

    def test_bringup_installs_world_and_depends_on_ground_runtime(self):
        cmake = (PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("DIRECTORY worlds", cmake)
        dependencies = {
            item.text for item in ET.parse(str(PACKAGE_XML)).getroot().findall(
                "exec_depend")}
        self.assertIn("ground_manipulator_runtime", dependencies)
        self.assertNotIn("bunker_sim_runtime", dependencies)

    def test_checker_rejects_stale_or_wrong_frame_sensor_data(self):
        checker = _load_checker()
        stamp = SimpleNamespace(to_sec=lambda: 9.8)
        valid = SimpleNamespace(
            header=SimpleNamespace(
                stamp=stamp, frame_id="uav1/camera_depth_frame"))
        result = checker.sensor_header_summary(
            valid, "uav1/camera_depth_frame", now=10.0)
        self.assertAlmostEqual(0.2, result["age_s"])
        slash_prefixed = SimpleNamespace(
            header=SimpleNamespace(
                stamp=stamp, frame_id="/uav1/camera_depth_frame"))
        self.assertEqual(
            "uav1/camera_depth_frame",
            checker.sensor_header_summary(
                slash_prefixed, "uav1/camera_depth_frame", now=10.0)["frame"])

        wrong = SimpleNamespace(
            header=SimpleNamespace(stamp=stamp, frame_id="camera_depth_frame"))
        with self.assertRaises(checker.RuntimeCheckError):
            checker.sensor_header_summary(
                wrong, "uav1/camera_depth_frame", now=10.0)
        with self.assertRaisesRegex(
                checker.RuntimeCheckError,
                r"stamp=9\.800000, now=12\.000000, age=2\.200s"):
            checker.sensor_header_summary(
                valid, "uav1/camera_depth_frame", now=12.0)

    def test_checker_keeps_one_sensor_subscription_for_two_frames(self):
        checker = _load_checker()
        checker_source = CHECKER.read_text(encoding="utf-8")

        def message(stamp):
            return SimpleNamespace(header=SimpleNamespace(
                stamp=SimpleNamespace(to_sec=lambda: stamp),
                frame_id="uav1/camera_link"))

        class Subscription:
            unregistered = False

            def unregister(self):
                self.unregistered = True

        subscription = Subscription()
        subscription_options = {}

        class FakeRospy:
            class Time:
                @staticmethod
                def now():
                    return SimpleNamespace(to_sec=lambda: 10.0)

            @staticmethod
            def is_shutdown():
                return False

            @staticmethod
            def Subscriber(_topic, _message_type, callback, **kwargs):
                subscription_options.update(kwargs)
                callback(message(9.7))
                callback(message(9.8))
                return subscription

        observed, summary = checker.wait_for_current_sensor(
            FakeRospy, "/camera", object(), "uav1/camera_link", 1.0)
        self.assertEqual(9.8, observed.header.stamp.to_sec())
        self.assertAlmostEqual(0.2, summary["age_s"])
        self.assertTrue(subscription.unregistered)
        self.assertEqual(1, subscription_options["queue_size"])
        self.assertEqual(4 * 1024 * 1024, subscription_options["buff_size"])
        self.assertTrue(subscription_options["tcp_nodelay"])
        self.assertIn("messages_lock = threading.Lock()", checker_source)
        self.assertIn("with messages_lock:", checker_source)

    def test_checker_rejects_non_finite_p450_odometry(self):
        checker = _load_checker()
        valid = SimpleNamespace(
            connected=True, odom_valid=True, position=[0.0, -0.1, 0.2])
        invalid = SimpleNamespace(
            connected=True, odom_valid=True,
            position=[0.0, float("nan"), 0.2])
        self.assertTrue(checker.valid_p450_state(valid))
        self.assertFalse(checker.valid_p450_state(invalid))

    def test_checker_requires_current_calibrated_camera_info(self):
        checker = _load_checker()
        stamp = SimpleNamespace(to_sec=lambda: 9.9)
        message = SimpleNamespace(
            header=SimpleNamespace(
                stamp=stamp, frame_id="uav1/camera_link"),
            width=640,
            height=480,
            K=[1.0] * 9,
            P=[1.0] * 12,
        )
        summary = checker.camera_info_summary(
            message, "uav1/camera_link", now=10.0)
        self.assertEqual(640, summary["width"])
        self.assertEqual(480, summary["height"])

        message.K[3] = float("nan")
        with self.assertRaises(checker.RuntimeCheckError):
            checker.camera_info_summary(
                message, "uav1/camera_link", now=10.0)

        contracts = checker.p450_sensor_contracts(
            "image", "camera_info", "imu")
        self.assertEqual(
            ["color", "color_info", "depth", "depth_info", "imu"],
            [contract[0] for contract in contracts])
        self.assertEqual(
            "/uav1/camera/color/camera_info", contracts[1][1])
        self.assertEqual(
            "/uav1/camera/depth/camera_info", contracts[3][1])

    def test_checker_requires_current_finite_map_tf_for_public_frames(self):
        checker = _load_checker()

        def transform(stamp=9.8, translation_x=0.1):
            return SimpleNamespace(
                header=SimpleNamespace(
                    stamp=SimpleNamespace(to_sec=lambda: stamp),
                    frame_id="map"),
                child_frame_id="uav1/base_link",
                transform=SimpleNamespace(
                    translation=SimpleNamespace(
                        x=translation_x, y=0.0, z=0.5),
                    rotation=SimpleNamespace(
                        x=0.0, y=0.0, z=0.0, w=1.0)))

        summary = checker.transform_summary(
            transform(), "map", "uav1/base_link", now=10.0)
        self.assertAlmostEqual(0.2, summary["age_s"])
        self.assertEqual("map<-uav1/base_link", summary["chain"])
        with self.assertRaises(checker.RuntimeCheckError):
            checker.transform_summary(
                transform(stamp=7.0), "map", "uav1/base_link", now=10.0)
        with self.assertRaises(checker.RuntimeCheckError):
            checker.transform_summary(
                transform(translation_x=float("inf")),
                "map", "uav1/base_link", now=10.0)

        self.assertIn("uav1/camera_imu_link", checker.AIR_TF_FRAMES)
        self.assertIn("uav1/camera_color_optical_frame", checker.AIR_TF_FRAMES)
        self.assertEqual({
            "ground/base_link",
            "ground/lidar_2d_link",
            "ground/aubo_i5_base_link",
            "ground/ee_link",
            "ground/d435_color_optical_frame",
            "ground/d435_depth_optical_frame",
            "ground/gripper_tcp_link",
            "ground/left_finger_pad",
            "ground/right_finger_pad",
        }, set(checker.GROUND_TF_FRAMES))

    def test_checker_requires_the_complete_ground_runtime(self):
        checker = _load_checker()
        source = CHECKER.read_text(encoding="utf-8")
        self.assertEqual("ground_robot", checker.GROUND_MODEL_NAME)
        self.assertEqual((1.0, 1.6), checker.GROUND_SCAN_FORWARD_RANGE_M)
        for required in (
                "ground.check_runtime_ready", "ground.check_controllers",
                "ground.current_joint_state", "ground.joint_state_summary",
                "ground.check_sensors", "ground.check_ground_scan",
                "bunker.check_motion"):
            self.assertIn(required, source)
        for forbidden in (
                "set_model_state", "teleport", "attach_link", "benchmark",
                "provenance"):
            self.assertNotIn(forbidden, source.lower())

    def test_checker_validates_ground_joint_state_immediately_after_read(self):
        checker = _load_checker()
        message = object()
        calls = []

        class FakeRospy:
            class Time:
                @staticmethod
                def now():
                    return SimpleNamespace(to_sec=lambda: 10.0)

        class FakeGround:
            @staticmethod
            def current_joint_state(rospy, joint_state_type, timeout):
                calls.append((rospy, joint_state_type, timeout))
                return message

            @staticmethod
            def joint_state_summary(observed, now):
                calls.append((observed, now))
                return {"age_s": 0.1}

        summary = checker.check_ground_joints(
            FakeRospy, FakeGround, object, 5.0)
        self.assertEqual({"age_s": 0.1}, summary)
        self.assertEqual((message, 10.0), calls[-1])

    def test_smoke_is_bounded_and_uses_the_platform_environment(self):
        source = SMOKE.read_text(encoding="utf-8")
        self.assertIn("scripts/with_p450_env.bash", source)
        self.assertIn("air_ground_standalone.launch", source)
        self.assertIn("check_air_ground_runtime.py", source)
        self.assertIn("/usr/bin/timeout", source)
        self.assertIn("rosparam get /use_sim_time", source)
        self.assertIn("/clock", source)
        self.assertIn('/bin/kill -INT "$launch_pid"', source)
        self.assertIn("checks_complete=false", source)
        self.assertIn(
            'if [[ "$status" -eq 0 && "$checks_complete" == true ]]; then',
            source)
        self.assertIn("PASS: P450 + Ground Robot shared-world smoke", source)
        self.assertEqual("checks_complete=true", source.rstrip().splitlines()[-1])
        self.assertNotIn("benchmark", source.lower())
        self.assertNotIn("evidence", source.lower())


if __name__ == "__main__":
    unittest.main()
