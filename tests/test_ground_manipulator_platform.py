#!/usr/bin/env python3

import importlib.util
import io
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET
from unittest import mock

import yaml


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src/platform/ground_manipulator_runtime"
PACKAGE_XML = PACKAGE / "package.xml"
CMAKE = PACKAGE / "CMakeLists.txt"
SETUP = PACKAGE / "setup.py"
RENDERER = PACKAGE / "src/ground_manipulator_runtime/renderer.py"
STARTUP = PACKAGE / "src/ground_manipulator_runtime/startup.py"
RENDER_SCRIPT = PACKAGE / "scripts/render_ground_robot.py"
SPAWN_SCRIPT = PACKAGE / "scripts/spawn_ground_robot.py"
ROBOT_XACRO = PACKAGE / "urdf/ground_robot.urdf.xacro"
D435_XACRO = PACKAGE / "urdf/d435.xacro"
CONTROLLERS = PACKAGE / "config/controllers.yaml"
RUNTIME_LAUNCH = PACKAGE / "launch/ground_robot_runtime.launch"
STANDALONE_LAUNCH = PACKAGE / "launch/ground_robot_standalone.launch"
WORLD = PACKAGE / "worlds/ground_robot.world"
AG95_CMAKE = ROOT / "src/vendor/dh_ag95_description/CMakeLists.txt"
MOVEIT_PACKAGE = ROOT / "src/ground/bunker_aubo_moveit_config"
MOVEIT_PACKAGE_XML = MOVEIT_PACKAGE / "package.xml"
GROUND_SRDF = MOVEIT_PACKAGE / "config/ground_robot.srdf"
GROUND_MOVEIT_CONTROLLERS = (
    MOVEIT_PACKAGE / "config/ground_controllers.yaml")
GROUND_MOVE_GROUP = MOVEIT_PACKAGE / "launch/ground_move_group.launch"
MOVEIT_EXECUTION_LAUNCH = (
    MOVEIT_PACKAGE / "launch/moveit_planning_execution.launch")
MOVEIT_EXECUTE_TEST = MOVEIT_PACKAGE / "test/moveit_execute.test"
LEGACY_SRDF = MOVEIT_PACKAGE / "config/bunker_aubo.srdf"
LEGACY_CONTROLLERS = MOVEIT_PACKAGE / "config/controllers.yaml"
GROUND_CHECKER = ROOT / "scripts/check_ground_manipulator_runtime.py"
GROUND_SMOKE = ROOT / "scripts/smoke_ground_manipulator_standalone.bash"


def _load_renderer():
    spec = importlib.util.spec_from_file_location(
        "ground_manipulator_renderer_test_target", str(RENDERER))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_render_cli():
    source_path = str(PACKAGE / "src")
    if source_path not in sys.path:
        sys.path.insert(0, source_path)
    spec = importlib.util.spec_from_file_location(
        "render_ground_robot_cli_test_target", str(RENDER_SCRIPT))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_startup():
    spec = importlib.util.spec_from_file_location(
        "ground_manipulator_startup_test_target", str(STARTUP))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_ground_checker():
    if not GROUND_CHECKER.is_file():
        raise AssertionError("ground manipulator runtime checker is missing")
    spec = importlib.util.spec_from_file_location(
        "check_ground_manipulator_runtime", str(GROUND_CHECKER))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class GroundManipulatorPlatformTest(unittest.TestCase):
    def test_ground_checker_rejects_stale_or_non_finite_joint_state(self):
        checker = _load_ground_checker()

        def joint_state(stamp=9.8, positions=None):
            if positions is None:
                positions = [0.0, -0.5, 1.0, 0.0, 1.0, 0.0, 0.1]
            return SimpleNamespace(
                header=SimpleNamespace(
                    stamp=SimpleNamespace(to_sec=lambda: stamp)),
                name=list(checker.ARM_JOINTS) + [checker.GRIPPER_JOINT],
                position=positions,
                velocity=[0.0] * 7,
                effort=[0.0] * 7)

        summary = checker.joint_state_summary(joint_state(), now=10.0)
        self.assertAlmostEqual(0.2, summary["age_s"])
        self.assertAlmostEqual(1.0, summary["positions"]["elbow_joint"])
        with self.assertRaises(checker.RuntimeCheckError):
            checker.joint_state_summary(joint_state(stamp=7.0), now=10.0)
        invalid = joint_state()
        invalid.position[2] = float("nan")
        with self.assertRaises(checker.RuntimeCheckError):
            checker.joint_state_summary(invalid, now=10.0)

    def test_ground_checker_requires_real_running_controllers(self):
        checker = _load_ground_checker()
        controllers = [
            SimpleNamespace(name=name, state="running", type="real/%s" % name)
            for name in checker.REQUIRED_CONTROLLERS
        ]
        summary = checker.controller_summary(controllers)
        self.assertEqual(
            sorted(checker.REQUIRED_CONTROLLERS),
            sorted(summary["running"]))
        controllers[-1].state = "stopped"
        with self.assertRaises(checker.RuntimeCheckError):
            checker.controller_summary(controllers)

    def test_ground_checker_bounds_goals_and_checks_final_positions(self):
        checker = _load_ground_checker()
        self.assertAlmostEqual(
            0.15,
            checker.bounded_joint_target(
                current=0.05, delta=0.10, lower=-3.04, upper=3.04,
                max_step=0.15))
        with self.assertRaises(checker.RuntimeCheckError):
            checker.bounded_joint_target(
                current=0.0, delta=0.4, lower=-3.04, upper=3.04,
                max_step=0.15)
        self.assertTrue(checker.positions_within(
            {"elbow_joint": 0.81}, {"elbow_joint": 0.8}, tolerance=0.02))
        self.assertFalse(checker.positions_within(
            {"elbow_joint": 0.9}, {"elbow_joint": 0.8}, tolerance=0.02))

    def test_ground_scan_checks_the_forward_landmark_not_arm_self_returns(
            self):
        checker = _load_ground_checker()
        sample_count = 720
        angle_min = -3.141592653589793
        increment = 2.0 * 3.141592653589793 / (sample_count - 1)
        ranges = [float("inf")] * sample_count
        ranges[20] = 0.22
        for index in range(sample_count):
            angle = angle_min + index * increment
            if abs(angle) <= 0.08:
                ranges[index] = 2.30
        message = SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(to_sec=lambda: 9.8),
                frame_id="ground/lidar_2d_link"),
            ranges=ranges,
            range_min=0.12,
            range_max=8.0,
            angle_min=angle_min,
            angle_increment=increment)
        summary = checker.ground_scan_summary(message, now=10.0)
        self.assertAlmostEqual(0.22, summary["nearest_range_m"])
        self.assertAlmostEqual(2.30, summary["forward_range_m"])
        for index in range(sample_count):
            angle = angle_min + index * increment
            if abs(angle) <= 0.08:
                ranges[index] = 1.30
        shared_summary = checker.ground_scan_summary(
            message, now=10.0, forward_range_bounds=(1.0, 1.6))
        self.assertAlmostEqual(1.30, shared_summary["forward_range_m"])
        for index in range(sample_count):
            angle = angle_min + index * increment
            if abs(angle) <= 0.08:
                ranges[index] = 0.3
        with self.assertRaises(checker.RuntimeCheckError):
            checker.ground_scan_summary(message, now=10.0)

    def test_ground_checker_uses_one_subscription_for_advancing_sensor_data(
            self):
        checker = _load_ground_checker()

        def message(stamp):
            return SimpleNamespace(header=SimpleNamespace(
                stamp=SimpleNamespace(to_sec=lambda: stamp),
                frame_id="ground/d435_color_optical_frame"))

        class Subscription:
            unregistered = False

            def unregister(self):
                self.unregistered = True

        subscription = Subscription()
        options = {}

        class FakeRospy:
            @staticmethod
            def is_shutdown():
                return False

            @staticmethod
            def Subscriber(_topic, _message_type, callback, **kwargs):
                options.update(kwargs)
                callback(message(9.7))
                callback(message(9.8))
                return subscription

        observed, summary = checker.wait_for_current_sensor(
            FakeRospy, "/ground/d435/color/image_raw", object(),
            lambda item: {"stamp": item.header.stamp.to_sec()}, 1.0)
        self.assertEqual(9.8, observed.header.stamp.to_sec())
        self.assertEqual(9.8, summary["stamp"])
        self.assertTrue(subscription.unregistered)
        self.assertEqual(1, options["queue_size"])
        self.assertGreaterEqual(options["buff_size"], 16 * 1024 * 1024)

    def test_ground_checker_keeps_depth_image_and_info_subscribed_together(
            self):
        checker = _load_ground_checker()
        callbacks = {}
        subscriptions = []

        def message(stamp):
            return SimpleNamespace(header=SimpleNamespace(
                stamp=SimpleNamespace(to_sec=lambda: stamp)))

        class Subscription:
            def __init__(self):
                self.unregistered = False

            def unregister(self):
                self.unregistered = True

        class FakeRospy:
            @staticmethod
            def is_shutdown():
                return False

            @staticmethod
            def Subscriber(topic, _message_type, callback, **_kwargs):
                callbacks[topic] = callback
                subscription = Subscription()
                subscriptions.append(subscription)
                if len(callbacks) == 2:
                    for stamp in (9.7, 9.8):
                        for receive in callbacks.values():
                            receive(message(stamp))
                return subscription

        contracts = (
            ("depth", "/depth/image", object(),
             lambda item: {"stamp": item.header.stamp.to_sec()}),
            ("depth_info", "/depth/camera_info", object(),
             lambda item: {"stamp": item.header.stamp.to_sec()}),
        )
        _messages, summaries = checker.wait_for_current_sensors(
            FakeRospy, contracts, 1.0)
        self.assertEqual(9.8, summaries["depth"]["stamp"])
        self.assertEqual(9.8, summaries["depth_info"]["stamp"])
        self.assertTrue(all(item.unregistered for item in subscriptions))

    def test_ground_checker_covers_public_runtime_without_shortcuts(self):
        checker = _load_ground_checker()
        source = GROUND_CHECKER.read_text(encoding="utf-8").lower()
        self.assertEqual("ground_robot", checker.MODEL_NAME)
        self.assertEqual({
            "joint_state_controller", "arm_controller", "gripper_controller",
        }, set(checker.REQUIRED_CONTROLLERS))
        self.assertIn("ground/gripper_tcp_link", checker.GROUND_TF_FRAMES)
        self.assertIn(
            "ground/d435_depth_optical_frame", checker.GROUND_TF_FRAMES)
        for text in (
                "/ground/runtime_ready", "/ground/d435/depth/points",
                "follow_joint_trajectory", "movegroupcommander",
                "bunker.check_motion"):
            self.assertIn(text, source)
        for forbidden in (
                "set_model_state", "teleport", "attach_link", "benchmark",
                "provenance"):
            self.assertNotIn(forbidden, source)

    def test_ground_smoke_is_bounded_and_reports_after_teardown(self):
        completed = subprocess.run(
            [str(GROUND_SMOKE), "--help"], cwd=ROOT, text=True,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            timeout=5, check=False)
        self.assertEqual(0, completed.returncode, completed.stderr)
        source = GROUND_SMOKE.read_text(encoding="utf-8")
        self.assertIn("moveit_planning_execution.launch", source)
        self.assertIn("check_ground_manipulator_runtime.py", source)
        self.assertIn("/usr/bin/timeout", source)
        self.assertIn("rosparam get /use_sim_time", source)
        self.assertIn("/clock", source)
        self.assertIn('/bin/kill -INT "$launch_pid"', source)
        checker = source.index("check_ground_manipulator_runtime.py")
        timeout = source.rfind("/usr/bin/timeout", 0, checker)
        self.assertGreater(timeout, source.index("setsid roslaunch"))
        self.assertGreater(source.index("PASS: Ground manipulator"),
                           source.index("with_bunker_env.bash"))
        self.assertNotIn("benchmark", source.lower())
        self.assertNotIn("evidence", source.lower())

    def test_moveit_layer_uses_ground_frames_and_real_namespaced_controllers(
            self):
        for path in (
                MOVEIT_PACKAGE_XML, GROUND_SRDF, GROUND_MOVEIT_CONTROLLERS,
                GROUND_MOVE_GROUP, MOVEIT_EXECUTION_LAUNCH,
                MOVEIT_EXECUTE_TEST):
            self.assertTrue(path.is_file(), "%s is missing" % path.name)
        self.assertFalse(LEGACY_SRDF.exists())
        self.assertFalse(LEGACY_CONTROLLERS.exists())

        package_root = ET.parse(str(MOVEIT_PACKAGE_XML)).getroot()
        dependencies = {
            item.text for item in package_root.findall("exec_depend")}
        self.assertTrue({
            "ground_manipulator_runtime", "moveit_kinematics",
            "moveit_planners_ompl", "moveit_ros_move_group",
            "moveit_ros_planning", "moveit_simple_controller_manager",
        }.issubset(dependencies))
        self.assertTrue({
            "bunker_aubo_description", "bunker_aubo_gazebo",
            "moveit_fake_controller_manager",
        }.isdisjoint(dependencies))

        srdf = ET.parse(str(GROUND_SRDF)).getroot()
        self.assertEqual("bunker_aubo", srdf.get("name"))
        chain = srdf.find("./group[@name='manipulator']/chain")
        self.assertEqual("ground/aubo_i5_base_link", chain.get("base_link"))
        self.assertEqual("ground/gripper_tcp_link", chain.get("tip_link"))
        end_effector = srdf.find("./end_effector[@name='ag95']")
        self.assertEqual("ground/ee_link", end_effector.get("parent_link"))
        for collision in srdf.findall("disable_collisions"):
            self.assertTrue(collision.get("link1").startswith("ground/"))
            self.assertTrue(collision.get("link2").startswith("ground/"))

        controllers = yaml.safe_load(
            GROUND_MOVEIT_CONTROLLERS.read_text(encoding="utf-8"))
        controller_list = controllers["controller_list"]
        self.assertEqual({
            "/ground/arm_controller", "/ground/gripper_controller",
        }, {item["name"] for item in controller_list})
        self.assertTrue(all(
            item["type"] == "FollowJointTrajectory"
            and item["action_ns"] == "follow_joint_trajectory"
            and item["default"]
            for item in controller_list))

        launch = ET.parse(str(GROUND_MOVE_GROUP)).getroot()
        description = launch.find("param[@name='robot_description']")
        self.assertIsNotNone(description)
        self.assertIn(
            "ground_manipulator_runtime)/scripts/render_ground_robot.py",
            description.get("command"))
        semantic = launch.find("param[@name='robot_description_semantic']")
        self.assertIsNotNone(semantic)
        self.assertIn("ground_robot.srdf", semantic.get("textfile"))
        node = launch.find("node[@name='move_group']")
        self.assertIsNotNone(node)
        remaps = {
            item.get("from"): item.get("to")
            for item in node.findall("remap")}
        self.assertEqual("/ground/joint_states", remaps.get("joint_states"))
        launch_text = GROUND_MOVE_GROUP.read_text(encoding="utf-8").lower()
        self.assertIn("ground_controllers.yaml", launch_text)
        for token in ("fake", "brick", "benchmark", "provenance"):
            self.assertNotIn(token, launch_text)

        runtime_text = RUNTIME_LAUNCH.read_text(encoding="utf-8").lower()
        self.assertNotIn("moveit", runtime_text)
        execution_text = MOVEIT_EXECUTION_LAUNCH.read_text(
            encoding="utf-8").lower()
        self.assertIn("ground_robot_standalone.launch", execution_text)
        self.assertIn("ground_move_group.launch", execution_text)
        self.assertNotIn("brick", execution_text)
        execution_launch = ET.parse(str(MOVEIT_EXECUTION_LAUNCH)).getroot()
        rviz = execution_launch.find("node[@name='rviz']")
        self.assertEqual(
            "/ground/joint_states",
            rviz.find("remap[@from='joint_states']").get("to"))
        execute_test = ET.parse(str(MOVEIT_EXECUTE_TEST)).getroot()
        test_node = execute_test.find("test[@test-name='moveit_execute']")
        self.assertEqual(
            "/ground/joint_states",
            test_node.find("remap[@from='joint_states']").get("to"))

    def test_startup_accepts_late_model_and_drives_home_with_controllers(self):
        self.assertTrue(STARTUP.is_file(), "startup module is missing")
        self.assertTrue(SPAWN_SCRIPT.is_file(), "startup CLI is missing")
        self.assertTrue(SPAWN_SCRIPT.stat().st_mode & 0o111)
        startup = _load_startup()

        events = []

        class SpawnAttempt:
            def __init__(self, result):
                self.spawn_result = result

            def result(self, timeout):
                events.append(("spawn_result", timeout))
                return self.spawn_result

        class Gateway:
            def __init__(self, spawn_result=False):
                self.spawn_result = spawn_result
                self.calls = []

            def begin_spawn(self, model_name, robot_xml, initial_pose):
                self.calls.append(
                    ("begin_spawn", model_name, robot_xml, initial_pose))
                return SpawnAttempt(self.spawn_result)

        gateway = Gateway(spawn_result=False)

        def wait_for_model(model_name, timeout):
            events.append(("wait", model_name, timeout))
            return True

        accepted = startup.initialize_ground_robot(
            gateway, "<robot name='ground'/>", object(), wait_for_model,
            lambda: events.append(("controllers",)),
            lambda positions, timeout: events.append(
                ("home", positions, timeout)) or True,
            timeout=25.0)
        self.assertFalse(accepted)
        self.assertEqual([
            ("wait", "ground_robot", 25.0),
            ("controllers",),
            ("home", startup.HOME_JOINT_POSITIONS, 25.0),
            ("spawn_result", 25.0),
        ], events)
        self.assertEqual("begin_spawn", gateway.calls[0][0])
        self.assertEqual({
            "shoulder_pan_joint": 0.0,
            "shoulder_lift_joint": -0.5,
            "elbow_joint": 1.0,
            "wrist_1_joint": 0.0,
            "wrist_2_joint": 1.0,
            "wrist_3_joint": 0.0,
            "left_outer_knuckle_joint": 0.0,
        }, dict(startup.HOME_JOINT_POSITIONS))
        stages = startup.arm_home_trajectory(
            startup.HOME_JOINT_POSITIONS[:-1])
        self.assertEqual([4.0, 8.0], [duration for duration, _ in stages])
        self.assertEqual(0.0, dict(stages[0][1])["shoulder_lift_joint"])
        self.assertEqual(1.0, dict(stages[0][1])["elbow_joint"])
        self.assertEqual(
            startup.HOME_JOINT_POSITIONS[:-1], stages[-1][1])

        complete_feedback = SimpleNamespace(
            name=[name for name, _value in startup.HOME_JOINT_POSITIONS],
            position=[value for _name, value in startup.HOME_JOINT_POSITIONS])
        self.assertTrue(startup.joint_feedback_is_complete(
            complete_feedback, startup.HOME_JOINT_POSITIONS))
        incomplete_feedback = SimpleNamespace(
            name=complete_feedback.name[:-1],
            position=complete_feedback.position[:-1])
        self.assertFalse(startup.joint_feedback_is_complete(
            incomplete_feedback, startup.HOME_JOINT_POSITIONS))
        non_finite_feedback = SimpleNamespace(
            name=complete_feedback.name,
            position=complete_feedback.position[:-1] + [float("nan")])
        self.assertFalse(startup.joint_feedback_is_complete(
            non_finite_feedback, startup.HOME_JOINT_POSITIONS))

        spawn_source = SPAWN_SCRIPT.read_text(encoding="utf-8")
        self.assertLess(
            spawn_source.index("wait_for_controller_feedback"),
            spawn_source.index("arm.send_goal"))

        with self.assertRaises(startup.StartupError):
            startup.initialize_ground_robot(
                Gateway(), "<robot/>", object(),
                lambda _name, _timeout: False, lambda: None,
                lambda _positions, _timeout: True, timeout=1.0)
        with self.assertRaises(startup.StartupError):
            startup.initialize_ground_robot(
                Gateway(), "<robot/>", object(),
                lambda _name, _timeout: True, lambda: None,
                lambda _positions, _timeout: False, timeout=1.0)

    def test_ag95_description_installs_runtime_model_resources(self):
        self.assertTrue(AG95_CMAKE.is_file())
        cmake = AG95_CMAKE.read_text(encoding="utf-8")
        self.assertIn(
            "install(DIRECTORY launch meshes urdf", cmake)
        self.assertIn(
            "DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}", cmake)

    def test_package_declares_only_runtime_robot_dependencies(self):
        self.assertTrue(PACKAGE_XML.is_file(), "runtime package is missing")
        root = ET.parse(str(PACKAGE_XML)).getroot()
        self.assertEqual("ground_manipulator_runtime", root.findtext("name"))
        dependencies = {
            item.text for tag in (
                "build_depend", "build_export_depend", "exec_depend")
            for item in root.findall(tag)
        }
        self.assertTrue({
            "aubo_description",
            "actionlib",
            "actionlib_msgs",
            "bunker_description",
            "bunker_sim_runtime",
            "controller_manager",
            "control_msgs",
            "dh_ag95_description",
            "gazebo_msgs",
            "gazebo_plugins",
            "gazebo_ros",
            "gazebo_ros_control",
            "geometry_msgs",
            "joint_state_controller",
            "position_controllers",
            "robot_state_publisher",
            "sensor_msgs",
            "std_msgs",
            "tf",
            "tf2_ros",
            "trajectory_msgs",
            "xacro",
        }.issubset(dependencies))
        self.assertNotIn("bunker_aubo_moveit_config", dependencies)
        manifest = PACKAGE_XML.read_text(encoding="utf-8").lower()
        for token in ("benchmark", "provenance", "pilot", "formal"):
            self.assertNotIn(token, manifest)

        self.assertTrue(CMAKE.is_file())
        cmake = CMAKE.read_text(encoding="utf-8")
        self.assertIn("catkin_python_setup()", cmake)
        self.assertIn("scripts/render_ground_robot.py", cmake)
        self.assertIn("scripts/spawn_ground_robot.py", cmake)
        self.assertIn("DIRECTORY config launch urdf worlds", cmake)
        self.assertTrue(SETUP.is_file())

    def test_renderer_canonicalizes_and_prefixes_only_link_references(self):
        self.assertTrue(RENDERER.is_file(), "renderer is missing")
        renderer = _load_renderer()
        root = ET.fromstring("""
<robot name="bunker_aubo">
  <link name="base_link"><collision name="old"/></link>
  <link name="wheel1.1_Link"/>
  <link name="aubo_i5_base_link"/>
  <link name="shoulder_link"/>
  <link name="ag95.body"/>
  <joint name="wheel1.1_jont" type="revolute">
    <parent link="base_link"/><child link="wheel1.1_Link"/>
    <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="0" velocity="0"/>
  </joint>
  <joint name="shoulder_pan_joint" type="revolute">
    <parent link="aubo_i5_base_link"/><child link="shoulder_link"/>
    <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <transmission name="arm_trans">
    <joint name="shoulder_pan_joint"/>
  </transmission>
  <gazebo reference="base_link"><mu1>0.5</mu1></gazebo>
  <gazebo reference="shoulder_pan_joint"><implicitSpringDamper>true</implicitSpringDamper></gazebo>
  <gazebo>
    <joint name="loop.joint" type="revolute">
      <parent>ag95.body</parent><child>shoulder_link</child>
    </joint>
    <plugin name="mimic" filename="mimic.so">
      <joint>wheel1.1_jont</joint>
      <mimicJoint>shoulder_pan_joint</mimicJoint>
    </plugin>
  </gazebo>
</robot>
""")

        renderer.transform_robot_tree(root)

        self.assertEqual(
            ["ground/base_link", "ground/wheel1_1_Link",
             "ground/aubo_i5_base_link", "ground/shoulder_link",
             "ground/ag95_body"],
            [link.get("name") for link in root.findall("link")])
        wheel = root.find("./joint[@name='wheel1_1_jont']")
        self.assertEqual("fixed", wheel.get("type"))
        self.assertEqual("ground/base_link", wheel.find("parent").get("link"))
        self.assertEqual(
            "ground/wheel1_1_Link", wheel.find("child").get("link"))
        self.assertIsNone(wheel.find("axis"))
        self.assertIsNone(wheel.find("limit"))
        arm = root.find("./joint[@name='shoulder_pan_joint']")
        self.assertEqual("revolute", arm.get("type"))
        self.assertEqual(
            "shoulder_pan_joint",
            root.find("./transmission/joint").get("name"))
        self.assertEqual(
            "ground/base_link",
            root.findall("gazebo")[0].get("reference"))
        self.assertEqual(
            "shoulder_pan_joint",
            root.findall("gazebo")[1].get("reference"))
        loop = root.find("./gazebo/joint[@name='loop_joint']")
        self.assertEqual("ground/ag95_body", loop.findtext("parent"))
        self.assertEqual("ground/shoulder_link", loop.findtext("child"))
        self.assertEqual(
            "wheel1_1_jont", root.findtext("./gazebo/plugin/joint"))
        self.assertEqual(
            "shoulder_pan_joint", root.findtext("./gazebo/plugin/mimicJoint"))

        base = root.find("./link[@name='ground/base_link']")
        collisions = base.findall("collision")
        self.assertEqual(1, len(collisions))
        self.assertEqual("base_link_collision", collisions[0].get("name"))
        self.assertEqual(
            renderer.BASE_COLLISION_SIZE,
            collisions[0].find("geometry/box").get("size"))

    def test_renderer_expands_and_serializes_a_xacro_file(self):
        renderer = _load_renderer()
        source = """<?xml version="1.0"?>
<robot name="bunker_aubo" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <link name="base_link"><collision name="old"/></link>
  <link name="wheel.1"/>
  <joint name="wheel.1.joint" type="revolute">
    <parent link="base_link"/><child link="wheel.1"/>
    <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="0" velocity="0"/>
  </joint>
</robot>
"""
        with tempfile.TemporaryDirectory(prefix="ground-render-") as directory:
            path = Path(directory) / "fixture.urdf.xacro"
            path.write_text(source, encoding="utf-8")
            payload = renderer.render_ground_robot(path, validate=False)
        root = ET.fromstring(payload)
        self.assertEqual("ground/base_link", root.find("link").get("name"))
        self.assertEqual("wheel_1_joint", root.find("joint").get("name"))
        self.assertTrue(payload.startswith("<?xml"))
        self.assertTrue(payload.endswith("\n"))

    def test_renderer_cli_is_quiet_and_reports_input_errors(self):
        self.assertTrue(RENDER_SCRIPT.is_file(), "renderer CLI is missing")
        self.assertTrue(RENDER_SCRIPT.stat().st_mode & 0o111)
        cli = _load_render_cli()
        output = io.StringIO()
        errors = io.StringIO()
        self.assertEqual(64, cli.main(argv=(), stdout=output, stderr=errors))
        self.assertEqual("", output.getvalue())
        self.assertIn("expected one xacro path", errors.getvalue())

        errors = io.StringIO()
        self.assertEqual(
            65,
            cli.main(
                argv=("/definitely/missing/ground.urdf.xacro",),
                stdout=output, stderr=errors))
        self.assertIn("xacro file is missing", errors.getvalue())

    def test_composite_model_has_only_robot_runtime_plugins(self):
        self.assertTrue(ROBOT_XACRO.is_file(), "composite xacro is missing")
        self.assertTrue(D435_XACRO.is_file(), "D435 xacro is missing")
        renderer = _load_renderer()
        ros_package_path = os.pathsep.join(
            (str(ROOT / "src"), "/opt/ros/noetic/share"))
        with mock.patch.dict(
                os.environ, {"ROS_PACKAGE_PATH": ros_package_path}):
            payload = renderer.render_ground_robot(ROBOT_XACRO)
        root = ET.fromstring(payload)

        links = root.findall("link")
        joints = root.findall("joint")
        link_names = {link.get("name") for link in links}
        joint_names = {joint.get("name") for joint in joints}
        self.assertTrue({
            "ground/base_link",
            "ground/aubo_i5_base_link",
            "ground/ee_link",
            "ground/ag95_base_link",
            "ground/gripper_tcp_link",
            "ground/d435_link",
            "ground/d435_color_optical_frame",
            "ground/d435_depth_optical_frame",
            "ground/lidar_2d_link",
        }.issubset(link_names))
        self.assertTrue({
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
            "left_outer_knuckle_joint",
        }.issubset(joint_names))
        child_links = {
            joint.find("child").get("link") for joint in joints
        }
        self.assertEqual({"ground/base_link"}, link_names - child_links)

        transmissions = {
            item.find("joint").get("name")
            for item in root.findall("transmission")
        }
        self.assertEqual({
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
            "left_outer_knuckle_joint",
        }, transmissions)

        plugin_libraries = {
            plugin.get("filename") for plugin in root.findall(".//plugin")
        }
        self.assertEqual({
            "libbunker_planar_move_plugin.so",
            "libgazebo_ros_laser.so",
            "libgazebo_ros_control.so",
            "libroboticsgroup_gazebo_mimic_joint_plugin.so",
            "libgazebo_ros_camera.so",
            "libgazebo_ros_openni_kinect.so",
        }, plugin_libraries)
        self.assertEqual(
            {"bunker_lidar_2d", "ground_d435_color", "ground_d435_depth"},
            {sensor.get("name") for sensor in root.findall(".//sensor")})
        camera_sensor_poses = {
            sensor.get("name"): sensor.findtext("pose")
            for sensor in root.findall(".//sensor")
            if sensor.get("name") in {
                "ground_d435_color", "ground_d435_depth"}}
        self.assertEqual({
            "ground_d435_color": (
                "0 0 0 0 -1.5707963267948966 1.5707963267948966"),
            "ground_d435_depth": (
                "0 0 0 0 -1.5707963267948966 1.5707963267948966"),
        }, camera_sensor_poses)
        camera_sensor_parents = {
            sensor.get("name"): gazebo.get("reference")
            for gazebo in root.findall("gazebo")
            for sensor in gazebo.findall("sensor")
            if sensor.get("name") in {
                "ground_d435_color", "ground_d435_depth"}}
        self.assertEqual({
            "ground_d435_color": "ground/d435_color_optical_frame",
            "ground_d435_depth": "ground/d435_depth_optical_frame",
        }, camera_sensor_parents)
        optical_joint_origins = {
            joint.get("name"): joint.find("origin").get("xyz")
            for joint in root.findall("joint")
            if joint.get("name") in {
                "d435_color_optical_joint",
                "d435_depth_optical_joint"}}
        self.assertEqual({
            "d435_color_optical_joint": "0.05 0 0",
            "d435_depth_optical_joint": "0.05 0 0",
        }, optical_joint_origins)
        for link_name in (
                "ground/d435_color_optical_frame",
                "ground/d435_depth_optical_frame"):
            inertial = root.find(
                "./link[@name='%s']/inertial" % link_name)
            self.assertIsNotNone(inertial)
            self.assertAlmostEqual(
                1e-3, float(inertial.find("mass").get("value")))
            inertia = inertial.find("inertia")
            for axis in ("ixx", "iyy", "izz"):
                self.assertGreater(float(inertia.get(axis)), 0.0)
        d435_joint_gazebo = root.find("./gazebo[@reference='d435_joint']")
        self.assertIsNotNone(d435_joint_gazebo)
        self.assertEqual(
            "true", d435_joint_gazebo.findtext("preserveFixedJoint"))
        for joint_name in (
                "d435_color_optical_joint", "d435_depth_optical_joint"):
            gazebo = root.find("./gazebo[@reference='%s']" % joint_name)
            self.assertIsNotNone(gazebo)
            self.assertEqual("true", gazebo.findtext("preserveFixedJoint"))
        color_plugin = root.find(
            ".//plugin[@name='ground_d435_color_controller']")
        self.assertEqual("/ground", color_plugin.findtext("robotNamespace"))
        self.assertEqual("d435/color", color_plugin.findtext("cameraName"))
        self.assertEqual(
            "image_raw", color_plugin.findtext("imageTopicName"))
        self.assertEqual(
            "camera_info",
            color_plugin.findtext("cameraInfoTopicName"))
        depth_plugin = root.find(
            ".//plugin[@name='ground_d435_depth_controller']")
        self.assertEqual("/ground", depth_plugin.findtext("robotNamespace"))
        self.assertEqual("d435/depth", depth_plugin.findtext("cameraName"))
        self.assertEqual(
            "image_raw",
            depth_plugin.findtext("depthImageTopicName"))
        self.assertEqual(
            "camera_info",
            depth_plugin.findtext("depthImageCameraInfoTopicName"))
        self.assertEqual(
            "points", depth_plugin.findtext("pointCloudTopicName"))
        lidar_joint = root.find("./joint[@name='lidar_2d_joint']")
        self.assertEqual(
            "0.45 0 0.25", lidar_joint.find("origin").get("xyz"))
        self.assertEqual(
            renderer.BASE_COLLISION_SIZE,
            root.find(
                "./link[@name='ground/base_link']/collision/geometry/box"
            ).get("size"))
        base_surface = root.find("./gazebo[@reference='ground/base_link']")
        self.assertEqual("0.0", base_surface.findtext("mu1"))
        self.assertEqual("0.0", base_surface.findtext("mu2"))

        lowered = payload.lower()
        for token in (
                "brick", "handoff", "attachment", "benchmark",
                "provenance", "lifecycle"):
            self.assertNotIn(token, lowered)

    def test_renderer_enforces_the_complete_runtime_tree(self):
        renderer = _load_renderer()
        ros_package_path = os.pathsep.join(
            (str(ROOT / "src"), "/opt/ros/noetic/share"))
        with mock.patch.dict(
                os.environ, {"ROS_PACKAGE_PATH": ros_package_path}), \
                mock.patch.object(
                    renderer, "validate_runtime_tree",
                    wraps=renderer.validate_runtime_tree) as validation:
            payload = renderer.render_ground_robot(ROBOT_XACRO)
        self.assertEqual(1, validation.call_count)

        root = ET.fromstring(payload)
        root.append(ET.fromstring(
            '<gazebo><plugin name="unexpected" filename="bad.so"/></gazebo>'))
        with self.assertRaises(renderer.RenderError):
            renderer.validate_runtime_tree(root)

    def test_worldless_launch_owns_ground_interfaces_and_real_controllers(self):
        for path in (CONTROLLERS, RUNTIME_LAUNCH, STANDALONE_LAUNCH, WORLD):
            self.assertTrue(path.is_file(), "%s is missing" % path.name)
        world = ET.parse(str(WORLD)).getroot().find("world")
        self.assertEqual("ground_robot_world", world.get("name"))
        self.assertNotEqual("ground_robot", world.get("name"))

        controllers = yaml.safe_load(CONTROLLERS.read_text(encoding="utf-8"))
        self.assertEqual({
            "joint_state_controller", "arm_controller", "gripper_controller",
            "gazebo_ros_control",
        }, set(controllers))
        self.assertEqual(
            "joint_state_controller/JointStateController",
            controllers["joint_state_controller"]["type"])
        self.assertEqual(
            "position_controllers/JointTrajectoryController",
            controllers["arm_controller"]["type"])
        self.assertEqual([
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
        ], controllers["arm_controller"]["joints"])
        self.assertAlmostEqual(
            0.10,
            controllers["arm_controller"]["constraints"]
            ["stopped_velocity_tolerance"])
        self.assertEqual(
            ["left_outer_knuckle_joint"],
            controllers["gripper_controller"]["joints"])
        self.assertEqual(
            {"p": 20.0, "i": 1.0, "d": 0.1, "i_clamp": 1.0},
            controllers["gazebo_ros_control"]["pid_gains"]
            ["right_outer_knuckle_joint"])

        runtime = ET.parse(str(RUNTIME_LAUNCH)).getroot()
        self.assertFalse(any(
            "empty_world.launch" in include.get("file", "")
            for include in runtime.findall(".//include")))
        group = runtime.find("group")
        self.assertIsNotNone(group)
        self.assertEqual("ground", group.get("ns"))
        nodes = {node.get("name"): node for node in group.findall("node")}
        self.assertEqual({
            "velocity_guard", "spawn_ground_robot", "controller_spawner",
            "robot_state_publisher",
        }, set(nodes))
        self.assertEqual(
            ("bunker_sim_runtime", "velocity_guard.py"),
            (nodes["velocity_guard"].get("pkg"),
             nodes["velocity_guard"].get("type")))
        spawn = nodes["spawn_ground_robot"]
        self.assertEqual(
            ("ground_manipulator_runtime", "spawn_ground_robot.py"),
            (spawn.get("pkg"), spawn.get("type")))
        self.assertEqual("true", spawn.get("required"))
        spawn_params = {
            item.get("name"): item.get("value")
            for item in spawn.findall("param")
        }
        self.assertEqual({
            "x", "y", "z", "roll", "pitch", "yaw", "model_timeout",
            "controller_timeout",
        }, set(spawn_params))
        self.assertEqual("true", nodes["controller_spawner"].get("required"))
        self.assertEqual(
            "--wait-for model_spawned joint_state_controller "
            "arm_controller gripper_controller",
            nodes["controller_spawner"].get("args"))
        self.assertFalse(any(
            node.get("pkg") == "tf2_ros" for node in nodes.values()))
        descriptions = [
            param for param in group.findall("param")
            if param.get("name") == "robot_description"]
        self.assertEqual(1, len(descriptions))
        self.assertIn(
            "ground_manipulator_runtime)/scripts/render_ground_robot.py",
            descriptions[0].get("command"))

        standalone = ET.parse(str(STANDALONE_LAUNCH)).getroot()
        localization = {
            node.get("name"): node.get("args")
            for node in standalone.findall("node")
        }
        self.assertEqual({
            "sim_world_to_map": "0 0 0 0 0 0 1 world map",
            "sim_localization_ground": "0 0 0 0 0 0 map ground/odom",
        }, localization)
        includes = standalone.findall("include")
        self.assertEqual(2, len(includes))
        self.assertEqual(1, sum(
            include.get("file") ==
            "$(find gazebo_ros)/launch/empty_world.launch"
            for include in includes))
        self.assertEqual(1, sum(
            include.get("file") ==
            "$(find ground_manipulator_runtime)/launch/ground_robot_runtime.launch"
            for include in includes))

        world = ET.parse(str(WORLD)).getroot().find("world")
        self.assertIsNotNone(world)
        self.assertIn(
            "model://ground_plane",
            [item.findtext("uri") for item in world.findall("include")])
        obstacle = world.find("./model[@name='ground_scan_obstacle']")
        self.assertIsNotNone(obstacle)
        self.assertEqual("3.0 0.0 0.5 0 0 0", obstacle.findtext("pose"))

        runtime_text = "\n".join(
            path.read_text(encoding="utf-8").lower()
            for path in (CONTROLLERS, RUNTIME_LAUNCH, STANDALONE_LAUNCH, WORLD))
        for token in (
                "brick", "handoff", "attachment", "benchmark",
                "provenance", "lifecycle"):
            self.assertNotIn(token, runtime_text)


if __name__ == "__main__":
    unittest.main()
