#!/usr/bin/env python3
"""Offline contract for the platform-owned P450 + D435 runtime."""

import copy
import hashlib
import json
import math
import os
import re
import shlex
import socket
import stat
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
RUNTIME_LAUNCH = PACKAGE_ROOT / "launch" / "p450_runtime.launch"
STANDALONE_LAUNCH = PACKAGE_ROOT / "launch" / "p450_standalone.launch"
TF_CONTRACT = PACKAGE_ROOT / "config" / "p450_tf_contract.yaml"
MID360_TF_CONTRACT = PACKAGE_ROOT / "config" / "p450_mid360_tf_contract.yaml"
SMOKE_SCRIPT = REPOSITORY_ROOT / "scripts" / "smoke_p450_standalone.bash"
MID360_SMOKE_SCRIPT = REPOSITORY_ROOT / "scripts" / "smoke_p450_mid360.bash"
SENSOR_TF_OFFSETS = (
    REPOSITORY_ROOT
    / "src/p450/prometheus_uav_control/launch/sensor_tf_offset.yaml"
)
UAV_ESTIMATOR = (
    REPOSITORY_ROOT
    / "src/p450/prometheus_uav_control/src/uav_estimator.cpp"
)

JINJA_MODEL = (
    REPOSITORY_ROOT
    / "src/p450/prometheus_gazebo/gazebo_models/uav_models"
    / "p450_D435i/p450_D435i.sdf.jinja"
)
MID360_JINJA_MODEL = (
    REPOSITORY_ROOT
    / "src/p450/prometheus_gazebo/gazebo_models/uav_models"
    / "p450_D435i_mid360/p450_D435i_mid360.sdf.jinja"
)
D435_MODEL = (
    REPOSITORY_ROOT
    / "src/p450/prometheus_gazebo/gazebo_models/sensor_models"
    / "D435i/model.sdf"
)
REALSENSE_PLUGIN = (
    REPOSITORY_ROOT
    / "src/p450/realsense_ros_gazebo/src/gazebo_ros_realsense.cpp"
)
PERCEPTION_CONFIG = (
    REPOSITORY_ROOT
    / "src/p450/brick_aerial_perception/config/perception.yaml"
)
LEGACY_SENSOR_LAUNCH = (
    REPOSITORY_ROOT
    / "src/p450/prometheus_gazebo/launch_uav_with_sensor"
    / "sitl_p450_d435i.launch"
)
LEGACY_TF_BRIDGE = (
    REPOSITORY_ROOT
    / "src/p450/brick_aerial_perception/scripts/world_tf_bridge.py"
)

EXPECTED_CONTROLLER_TRANSLATION = (
    0.13614773689465415,
    0.0,
    0.11272472554168546,
)
EXPECTED_CONTROLLER_RPY = (
    -1.9207963267948966,
    0.0,
    -1.5707963267948966,
)
EXPECTED_CONTROLLER_QUATERNION = (
    -0.5794173382492648,
    0.5794173382492647,
    -0.4053092006556687,
    0.4053092006556688,
)
EXPECTED_LIDAR_MOUNT = (0.13, 0.0, 0.23, 0.0, 0.35, 0.0)
EXPECTED_LIDAR_SENSOR_LOCAL = (0.0, 0.0, 0.05, 0.0, 0.0, 0.0)
EXPECTED_LIDAR_TRANSLATION = (
    0.14714489037277257,
    0.0,
    0.27696863564236896,
)
EXPECTED_LIDAR_RPY = (0.0, 0.35, 0.0)
EXPECTED_LIDAR_QUATERNION = (
    0.0,
    math.sin(0.175),
    0.0,
    math.cos(0.175),
)
EXPECTED_STATIC_TRANSFORMS = {
    "uav1/camera_depth_frame": (
        "uav1/camera_link", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
    "uav1/camera_ired1_frame": (
        "uav1/camera_link", (-0.03, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
    "uav1/camera_ired2_frame": (
        "uav1/camera_link", (0.03, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
    "uav1/camera_imu_link": (
        "uav1/camera_link", (0.0, 0.12, 0.0), (0.5, -0.5, 0.5, 0.5)),
    "uav1/d435i_link": (
        "uav1/camera_link", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
    "uav1/camera_color_optical_frame": (
        "uav1/camera_link", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
    "uav1/camera_depth_optical_frame": (
        "uav1/camera_depth_frame", (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 1.0)),
}


def _require_file(path):
    if not path.is_file():
        raise AssertionError("required artifact is missing: %s" % path)
    return path


def _read(path):
    return _require_file(path).read_text(encoding="utf-8")


def _xml(path):
    return ET.parse(str(_require_file(path))).getroot()


def _yaml(path):
    with _require_file(path).open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _nodes(root, package=None, node_type=None):
    nodes = root.findall(".//node")
    if package is not None:
        nodes = [node for node in nodes if node.get("pkg") == package]
    if node_type is not None:
        nodes = [node for node in nodes if node.get("type") == node_type]
    return nodes


def _one(nodes, description):
    if len(nodes) != 1:
        raise AssertionError(
            "expected exactly one %s, found %d" % (description, len(nodes)))
    return nodes[0]


def _param_map(node):
    parameters = node.findall("./param")
    names = [parameter.get("name") for parameter in parameters]
    if len(names) != len(set(names)):
        raise AssertionError("duplicate parameter names on node %s" % node.get("name"))
    return {parameter.get("name"): parameter for parameter in parameters}


def _floats(values):
    return tuple(float(value) for value in values)


def _assert_float_tuple(test, actual, expected, places=12):
    test.assertEqual(len(expected), len(actual))
    for actual_value, expected_value in zip(actual, expected):
        test.assertAlmostEqual(expected_value, actual_value, places=places)


def _quaternion_from_rpy(roll, pitch, yaw):
    cr = math.cos(roll / 2.0)
    sr = math.sin(roll / 2.0)
    cp = math.cos(pitch / 2.0)
    sp = math.sin(pitch / 2.0)
    cy = math.cos(yaw / 2.0)
    sy = math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class ArtifactMaterializationTest(unittest.TestCase):
    def test_all_task5_artifacts_exist(self):
        missing = [
            str(path.relative_to(REPOSITORY_ROOT))
            for path in (RUNTIME_LAUNCH, STANDALONE_LAUNCH, TF_CONTRACT,
                         MID360_TF_CONTRACT, SMOKE_SCRIPT)
            if not path.is_file()
        ]
        self.assertEqual([], missing)

    def test_smoke_entrypoint_is_executable(self):
        _require_file(SMOKE_SCRIPT)
        self.assertTrue(os.access(str(SMOKE_SCRIPT), os.X_OK))


class RuntimeLaunchContractTest(unittest.TestCase):
    def setUp(self):
        self.root = _xml(RUNTIME_LAUNCH)
        self.source = _read(RUNTIME_LAUNCH)

    def test_runtime_is_worldless_and_fixed_to_one_uav1(self):
        self.assertEqual("launch", self.root.tag)
        self.assertEqual([], self.root.findall(".//include"))
        self.assertEqual([], _nodes(self.root, "gazebo_ros", "gzserver"))
        self.assertEqual([], _nodes(self.root, "gazebo_ros", "gzclient"))

        arguments = {arg.get("name"): arg.get("default")
                     for arg in self.root.findall("./arg")}
        self.assertEqual({
            "use_sim_time": "true",
            "uav1_init_x": "0.0",
            "uav1_init_y": "0.0",
            "uav1_init_z": "0.15",
            "uav1_init_yaw": "0.0",
            "px4_workdir": "sitl_amov_0",
            "enable_mid360": "false",
        }, arguments)
        use_sim_time = _one(
            [param for param in self.root.findall("./param")
             if param.get("name") == "/use_sim_time"],
            "/use_sim_time parameter")
        self.assertEqual("bool", use_sim_time.get("type"))
        self.assertEqual("$(arg use_sim_time)", use_sim_time.get("value"))

        groups = self.root.findall("./group")
        self.assertEqual(["/uav1"], [group.get("ns") for group in groups])
        self.assertNotIn("world_tf_bridge.py", self.source)

    def test_model_is_rendered_and_spawned_exactly_once(self):
        group = _one(self.root.findall("./group"), "uav1 group")
        sdf_parameters = [
            param for param in group.findall("./param")
            if param.get("name") == "sdf_p450_D435i_0"
        ]
        self.assertEqual(2, len(sdf_parameters))
        default_parameter = _one(
            [param for param in sdf_parameters
             if param.get("unless") == "$(arg enable_mid360)"],
            "default rendered P450 SDF parameter")
        mid360_parameter = _one(
            [param for param in sdf_parameters
             if param.get("if") == "$(arg enable_mid360)"],
            "MID360 rendered P450 SDF parameter")
        self.assertIsNone(default_parameter.get("if"))
        self.assertIsNone(mid360_parameter.get("unless"))
        self.assertEqual(
            "/usr/bin/python3 "
            "$(find prometheus_gazebo)/scripts/jinja_gen.py --stdout "
            "--mavlink_id=1 --mavlink_udp_port=14560 "
            "--mavlink_tcp_port=4560 "
            "$(find prometheus_gazebo)/gazebo_models/uav_models/"
            "p450_D435i/p450_D435i.sdf.jinja "
            "$(find prometheus_gazebo)",
            default_parameter.get("command"),
        )
        self.assertEqual(
            "/usr/bin/python3 "
            "$(find prometheus_gazebo)/scripts/jinja_gen.py --stdout "
            "--mavlink_id=1 --mavlink_udp_port=14560 "
            "--mavlink_tcp_port=4560 "
            "$(find prometheus_gazebo)/gazebo_models/uav_models/"
            "p450_D435i_mid360/p450_D435i_mid360.sdf.jinja "
            "$(find prometheus_gazebo)",
            mid360_parameter.get("command"),
        )

        spawns = _nodes(self.root, "gazebo_ros", "spawn_model")
        spawn = _one(spawns, "P450 spawn node")
        self.assertEqual("p450_D435i_1_spawn", spawn.get("name"))
        self.assertEqual(
            "-sdf -param sdf_p450_D435i_0 -model p450_D435i_0 "
            "-x $(arg uav1_init_x) -y $(arg uav1_init_y) "
            "-z $(arg uav1_init_z) -Y $(arg uav1_init_yaw)",
            spawn.get("args"),
        )

        default_model = _read(JINJA_MODEL)
        mid360_model = _read(MID360_JINJA_MODEL)
        self.assertNotIn("liblivox_laser_gazebo_plugins.so", default_model)
        self.assertEqual(1, mid360_model.count("model://D435i"))
        self.assertEqual(
            1, mid360_model.count("librealsense_gazebo_plugin.so"))
        self.assertEqual(
            1, mid360_model.count("liblivox_laser_gazebo_plugins.so"))

    def test_px4_uses_only_the_platform_wrapper(self):
        px4 = _one(
            _nodes(self.root, "sim_platform_bringup", "px4_sitl_node.bash"),
            "platform PX4 wrapper node")
        self.assertEqual("sitl_1", px4.get("name"))
        self.assertEqual("ROS_HOME", px4.get("cwd"))
        self.assertEqual(
            ["$(find", "px4)/ROMFS/px4fmu_common", "-s",
             "etc/init.d-posix/rcS", "-i", "0", "-w",
             "$(arg", "px4_workdir)",
             "-d"],
            shlex.split(px4.get("args")),
        )
        environment = {item.get("name"): item.get("value")
                       for item in px4.findall("./env")}
        self.assertEqual({
            "PX4_SIM_MODEL": "p450",
            "PX4_ESTIMATOR": "ekf2",
            "PX4_SIM_SPEED_FACTOR": "1.0",
        }, environment)
        self.assertEqual([], _nodes(self.root, "px4", "px4"))

    def test_mavros_ports_identity_and_tf_override_are_exact(self):
        mavros = _one(_nodes(self.root, "mavros", "mavros_node"), "MAVROS")
        self.assertEqual("mavros", mavros.get("name"))
        parameters = _param_map(mavros)
        expected_values = {
            "fcu_url": "udp://:14540@localhost:14580",
            "gcs_url": "",
            "target_system_id": "1",
            "target_component_id": "1",
            "local_position/tf/send": "false",
        }
        self.assertEqual(expected_values,
                         {name: parameters[name].get("value")
                          for name in expected_values})
        self.assertEqual("bool", parameters["local_position/tf/send"].get("type"))

        children = list(mavros)
        loads = [child for child in children if child.tag == "rosparam"]
        self.assertEqual([
            "$(find prometheus_gazebo)/config/mavros_config/"
            "px4_config_outdoor.yaml",
            "$(find prometheus_gazebo)/config/mavros_config/"
            "px4_pluginlists_outdoor.yaml",
        ], [load.get("file") for load in loads])
        self.assertTrue(all(load.get("command") == "load" for load in loads))
        self.assertGreater(
            children.index(parameters["local_position/tf/send"]),
            max(children.index(load) for load in loads),
        )

    def test_runtime_uses_uav_odom_and_owns_no_global_anchor(self):
        controller = _one(
            _nodes(self.root, "prometheus_uav_control", "uav_control_main"),
            "Prometheus UAV controller")
        parameters = _param_map(controller)
        self.assertEqual(
            "uav1/odom",
            parameters["tf_parent_frame"].get("value"),
        )
        static_nodes = _nodes(
            self.root, "tf2_ros", "static_transform_publisher")
        self.assertTrue(all(
            " world " not in " %s " % node.get("args", "")
            and " map uav1/odom" not in node.get("args", "")
            for node in static_nodes))

        estimator = _read(UAV_ESTIMATOR)
        self.assertIn(
            'nh.param<std::string>("tf_parent_frame", tf_parent_frame, '
            '"world")', estimator)
        self.assertIn("tfs.header.frame_id = tf_parent_frame;", estimator)
        self.assertNotIn('tfs.header.frame_id = "world"', estimator)

    def test_controller_load_order_and_exact_model_offsets(self):
        controller = _one(
            _nodes(self.root, "prometheus_uav_control", "uav_control_main"),
            "Prometheus UAV controller")
        self.assertEqual("uav_control_main_1", controller.get("name"))
        self.assertIs(controller, _one(
            [node for node in self.root.findall("./node")
             if node.get("name") == "uav_control_main_1"],
            "root controller"))
        self.assertEqual([], _nodes(self.root, "prometheus_uav_control", "joy_node"))

        children = list(controller)
        loads = [child for child in children if child.tag == "rosparam"]
        self.assertEqual([
            "$(find prometheus_uav_control)/launch/uav_control_outdoor.yaml",
            "$(find prometheus_uav_control)/launch/sensor_tf_offset.yaml",
        ], [load.get("file") for load in loads])
        parameters = _param_map(controller)
        expected_values = {
            "uav_id": "1",
            "sim_mode": "true",
            "flag_printf": "false",
            "control/enable_external_control": "false",
            "D435i/offset_x": str(EXPECTED_CONTROLLER_TRANSLATION[0]),
            "D435i/offset_y": str(EXPECTED_CONTROLLER_TRANSLATION[1]),
            "D435i/offset_z": str(EXPECTED_CONTROLLER_TRANSLATION[2]),
            "D435i/offset_roll": str(EXPECTED_CONTROLLER_RPY[0]),
            "D435i/offset_pitch": str(EXPECTED_CONTROLLER_RPY[1]),
            "D435i/offset_yaw": str(EXPECTED_CONTROLLER_RPY[2]),
        }
        self.assertEqual(expected_values,
                         {name: parameters[name].get("value")
                          for name in expected_values})
        last_load_index = max(children.index(load) for load in loads)
        for name in expected_values:
            self.assertGreater(children.index(parameters[name]), last_load_index)

        lidar_parameters = {
            parameter.get("name"): parameter
            for parameter in controller.findall("./param")
            if parameter.get("name", "").startswith("Lidar/offset_")
        }
        expected_lidar_values = {
            "Lidar/offset_x": "0.14714489037277257",
            "Lidar/offset_y": "0.0",
            "Lidar/offset_z": "0.27696863564236896",
            "Lidar/offset_roll": "0.0",
            "Lidar/offset_pitch": "0.35",
            "Lidar/offset_yaw": "0.0",
        }
        self.assertEqual(set(expected_lidar_values), set(lidar_parameters))
        for name, expected in expected_lidar_values.items():
            parameter = lidar_parameters[name]
            self.assertEqual(expected, parameter.get("value"), name)
            self.assertEqual("double", parameter.get("type"), name)
            self.assertEqual("$(arg enable_mid360)", parameter.get("if"), name)
            self.assertIsNone(parameter.get("unless"), name)
            self.assertGreater(children.index(parameter), last_load_index)
        self.assertNotIn("sensor_tf_offset_mid360.yaml", self.source)

        inherited_offsets = _yaml(SENSOR_TF_OFFSETS)
        self.assertEqual({
            "offset_x": 0.0,
            "offset_y": 0.0,
            "offset_z": 0.0,
            "offset_roll": 0.0,
            "offset_pitch": 0.0,
            "offset_yaw": 0.0,
        }, inherited_offsets["Lidar"])

    def test_controller_disables_startup_px4_parameter_mutation(self):
        controller = _one(
            _nodes(self.root, "prometheus_uav_control", "uav_control_main"),
            "Prometheus UAV controller")
        children = list(controller)
        loads = [child for child in children if child.tag == "rosparam"]
        self.assertTrue(loads)

        parameters = _param_map(controller)
        self.assertIn("enable_px4_params_load", parameters)
        policy = parameters["enable_px4_params_load"]
        self.assertEqual("bool", policy.get("type"))
        self.assertEqual("false", policy.get("value"))
        self.assertGreater(
            children.index(policy),
            max(children.index(load) for load in loads),
            "the local no-mutation policy must override inherited YAML",
        )

    def test_static_tf_nodes_match_the_unique_authority_matrix(self):
        nodes = _nodes(self.root, "tf2_ros", "static_transform_publisher")
        self.assertEqual(7, len(nodes))
        actual = {}
        for node in nodes:
            arguments = shlex.split(node.get("args"))
            self.assertEqual(9, len(arguments))
            translation = _floats(arguments[:3])
            rotation = _floats(arguments[3:7])
            parent, child = arguments[7:]
            self.assertNotIn(child, actual)
            actual[child] = (parent, translation, rotation)
        expected = EXPECTED_STATIC_TRANSFORMS
        self.assertEqual(set(expected), set(actual))
        self.assertNotIn("uav1/lidar_link", actual)
        for child, transform in expected.items():
            self.assertEqual(transform[0], actual[child][0])
            _assert_float_tuple(self, actual[child][1], transform[1])
            _assert_float_tuple(self, actual[child][2], transform[2])


class StandaloneLaunchContractTest(unittest.TestCase):
    def setUp(self):
        self.root = _xml(STANDALONE_LAUNCH)

    def test_standalone_owns_one_world_and_includes_one_runtime(self):
        localization_nodes = {
            node.get("name"): node
            for node in self.root.findall("./node")
        }
        self.assertEqual({"sim_world_to_map", "sim_localization_uav1"},
                         set(localization_nodes))
        self.assertEqual(
            "0 0 0 0 0 0 1 world map",
            localization_nodes["sim_world_to_map"].get("args"))
        self.assertEqual(
            "$(arg uav1_init_x) $(arg uav1_init_y) $(arg uav1_init_z) "
            "$(arg uav1_init_yaw) 0 0 map uav1/odom",
            localization_nodes["sim_localization_uav1"].get("args"))
        arguments = {arg.get("name"): arg.get("default")
                     for arg in self.root.findall("./arg")}
        self.assertEqual({
            "gui": "true",
            "use_sim_time": "true",
            "world": "$(find prometheus_gazebo)/gazebo_worlds/"
                     "prometheus_empty.world",
            "uav1_init_x": "0.0",
            "uav1_init_y": "0.0",
            "uav1_init_z": "0.15",
            "uav1_init_yaw": "0.0",
            "px4_workdir": "sitl_amov_0",
            "enable_mid360": "false",
        }, arguments)
        includes = self.root.findall("./include")
        self.assertEqual([
            "$(find gazebo_ros)/launch/empty_world.launch",
            "$(find sim_platform_bringup)/launch/p450_runtime.launch",
        ], [include.get("file") for include in includes])

        world_args = {arg.get("name"): arg.get("value")
                      for arg in includes[0].findall("./arg")}
        self.assertEqual({
            "world_name": "$(arg world)",
            "gui": "$(arg gui)",
            "use_sim_time": "$(arg use_sim_time)",
        }, world_args)
        runtime_args = {arg.get("name"): arg.get("value")
                        for arg in includes[1].findall("./arg")}
        self.assertEqual({
            "use_sim_time": "$(arg use_sim_time)",
            "uav1_init_x": "$(arg uav1_init_x)",
            "uav1_init_y": "$(arg uav1_init_y)",
            "uav1_init_z": "$(arg uav1_init_z)",
            "uav1_init_yaw": "$(arg uav1_init_yaw)",
            "px4_workdir": "$(arg px4_workdir)",
            "enable_mid360": "$(arg enable_mid360)",
        }, runtime_args)


class Mid360TfContractTest(unittest.TestCase):
    def setUp(self):
        self.contract = _yaml(MID360_TF_CONTRACT)

    def test_contract_freezes_model_composite_authority_and_message(self):
        self.assertEqual({
            "schema_version", "profile", "frame_normalization", "model",
            "composite", "topic", "runtime_tolerance",
        }, set(self.contract))
        self.assertEqual(1, self.contract["schema_version"])
        self.assertEqual("p450_D435i_mid360", self.contract["profile"])
        self.assertEqual("strip_leading_slash",
                         self.contract["frame_normalization"])
        model = self.contract["model"]
        self.assertEqual({"mount", "ray_sensor"}, set(model))
        _assert_float_tuple(
            self, tuple(model["mount"]["translation_m"]),
            EXPECTED_LIDAR_MOUNT[:3])
        _assert_float_tuple(
            self, tuple(model["mount"]["rpy_rad"]),
            EXPECTED_LIDAR_MOUNT[3:])
        _assert_float_tuple(
            self, tuple(model["ray_sensor"]["translation_m"]),
            EXPECTED_LIDAR_SENSOR_LOCAL[:3])
        _assert_float_tuple(
            self, tuple(model["ray_sensor"]["rpy_rad"]),
            EXPECTED_LIDAR_SENSOR_LOCAL[3:])

        composite = self.contract["composite"]
        self.assertEqual("uav1/base_link", composite["parent"])
        self.assertEqual("uav1/lidar_link", composite["child"])
        self.assertEqual("/uav_control_main_1", composite["authority"])
        _assert_float_tuple(
            self, tuple(composite["translation_m"]),
            EXPECTED_LIDAR_TRANSLATION)
        _assert_float_tuple(
            self, tuple(composite["rpy_rad"]), EXPECTED_LIDAR_RPY)
        _assert_float_tuple(
            self, tuple(composite["rotation_xyzw"]),
            EXPECTED_LIDAR_QUATERNION)
        self.assertEqual({
            "name": "/uav1/livox/lidar",
            "type": "prometheus_msgs/LivoxCustomMsg",
        }, self.contract["topic"])
        self.assertEqual({
            "translation_m": 1.0e-6,
            "rotation_xyzw": 1.0e-6,
        }, self.contract["runtime_tolerance"])

        # The controller loads these values as float32, so future live checks
        # must compare the published transform with the recorded tolerance.
        estimator = _read(UAV_ESTIMATOR)
        for suffix in (
                "x", "y", "z", "roll", "pitch", "yaw"):
            self.assertIn(
                'nh.param<float>("Lidar/offset_%s"' % suffix,
                estimator,
            )

    def test_composite_is_derived_from_mount_and_ray_sensor(self):
        model = self.contract["model"]
        mount_t = tuple(model["mount"]["translation_m"])
        mount_rpy = tuple(model["mount"]["rpy_rad"])
        sensor_t = tuple(model["ray_sensor"]["translation_m"])
        sensor_rpy = tuple(model["ray_sensor"]["rpy_rad"])
        pitch = mount_rpy[1]
        derived_translation = (
            mount_t[0] + math.sin(pitch) * sensor_t[2],
            mount_t[1] + sensor_t[1],
            mount_t[2] + math.cos(pitch) * sensor_t[2],
        )
        derived_rpy = tuple(
            mount_rpy[index] + sensor_rpy[index] for index in range(3))
        composite = self.contract["composite"]
        _assert_float_tuple(
            self, derived_translation, tuple(composite["translation_m"]))
        _assert_float_tuple(
            self, derived_rpy, tuple(composite["rpy_rad"]))
        _assert_float_tuple(
            self, _quaternion_from_rpy(*derived_rpy),
            tuple(composite["rotation_xyzw"]))


class TfContractTest(unittest.TestCase):
    def setUp(self):
        self.contract = _yaml(TF_CONTRACT)

    def test_dynamic_and_static_children_have_one_canonical_authority(self):
        self.assertEqual(1, self.contract["schema_version"])
        self.assertEqual("strip_leading_slash",
                         self.contract["frame_normalization"])
        transforms = self.contract["transforms"]
        dynamic = transforms["dynamic"]
        self.assertEqual([
            ("uav1/odom", "uav1/base_link", "/uav_control_main_1"),
            ("uav1/base_link", "uav1/camera_link", "/uav_control_main_1"),
        ], [(item["parent"], item["child"], item["authority"])
            for item in dynamic])
        _assert_float_tuple(
            self, tuple(dynamic[1]["translation_m"]),
            EXPECTED_CONTROLLER_TRANSLATION)
        _assert_float_tuple(
            self, tuple(dynamic[1]["rotation_xyzw"]),
            EXPECTED_CONTROLLER_QUATERNION)

        static = transforms["static"]
        children = [item["child"] for item in static]
        self.assertEqual(len(children), len(set(children)))
        self.assertEqual(set(EXPECTED_STATIC_TRANSFORMS), set(children))
        all_frames = [item[key] for family in (dynamic, static)
                      for item in family for key in ("parent", "child")]
        self.assertTrue(all(not frame.startswith("/") for frame in all_frames))

        for item in static:
            parent, translation, rotation = EXPECTED_STATIC_TRANSFORMS[item["child"]]
            self.assertEqual(parent, item["parent"])
            _assert_float_tuple(self, tuple(item["translation_m"]), translation)
            _assert_float_tuple(self, tuple(item["rotation_xyzw"]), rotation)
            self.assertAlmostEqual(
                1.0,
                math.sqrt(sum(value * value
                              for value in item["rotation_xyzw"])),
                places=12,
            )

    def test_controller_contract_matches_model_derived_composite(self):
        controller = self.contract["controller"]
        self.assertEqual("/uav_control_main_1", controller["node"])
        offset = controller["d435i_offset"]
        _assert_float_tuple(
            self, tuple(offset["translation_m"]), EXPECTED_CONTROLLER_TRANSLATION)
        _assert_float_tuple(
            self, tuple(offset["rpy_rad"]), EXPECTED_CONTROLLER_RPY)


class ImportedSensorEvidenceTest(unittest.TestCase):
    def test_model_geometry_recomputes_the_controller_camera_transform(self):
        jinja = _read(JINJA_MODEL)
        include = re.search(
            r"<include>\s*<uri>model://D435i</uri>\s*"
            r"<pose>([^<]+)</pose>.*?</include>",
            jinja,
            flags=re.DOTALL,
        )
        self.assertIsNotNone(include)
        mount = _floats(include.group(1).split())
        self.assertEqual((0.095, 0.0, 0.0, 0.0, 0.35, 0.0), mount)

        model = _xml(D435_MODEL)
        poses = {
            link.get("name"): _floats(link.findtext("pose").split())
            for link in model.findall("./model/link")
            if link.find("pose") is not None
        }
        self.assertEqual((0.0, 0.0, 0.12, 0.0, 0.0, 0.0),
                         poses["camera_color_frame"])
        self.assertEqual((0.0, 0.0, 0.12, 0.0, 0.0, 0.0),
                         poses["camera_depth_frame"])
        self.assertEqual((0.0, 0.03, 0.12, 0.0, 0.0, 0.0),
                         poses["camera_ired1_frame"])
        self.assertEqual((0.0, -0.03, 0.12, 0.0, 0.0, 0.0),
                         poses["camera_ired2_frame"])

        pitch = mount[4]
        sensor_z = poses["camera_color_frame"][2]
        translation = (
            mount[0] + math.sin(pitch) * sensor_z,
            mount[1],
            mount[2] + math.cos(pitch) * sensor_z,
        )
        rpy = (-math.pi / 2.0 - pitch, 0.0, -math.pi / 2.0)
        _assert_float_tuple(self, translation, EXPECTED_CONTROLLER_TRANSLATION)
        _assert_float_tuple(self, rpy, EXPECTED_CONTROLLER_RPY)
        _assert_float_tuple(
            self,
            _quaternion_from_rpy(*rpy),
            EXPECTED_CONTROLLER_QUATERNION,
        )

    def test_plugin_message_frames_and_perception_alias_are_connected(self):
        jinja = _read(JINJA_MODEL)
        expected_frames = {
            "colorOpticalframeName": "uav1/camera_link",
            "depthOpticalframeName": "uav1/camera_depth_frame",
            "infrared1OpticalframeName": "uav1/camera_ired1_frame",
            "infrared2OpticalframeName": "uav1/camera_ired2_frame",
            "frameName": "uav1/camera_imu_link",
        }
        for tag, expected in expected_frames.items():
            values = re.findall(r"<%s>([^<]+)</%s>" % (tag, tag), jinja)
            normalized = {
                value.replace("{{ mavlink_id }}", "1").lstrip("/")
                for value in values
            }
            self.assertIn(expected, normalized, msg=tag)

        expected_topics = {
            "depthTopicName": "/uav1/camera/depth/image_raw",
            "depthCameraInfoTopicName": "/uav1/camera/depth/camera_info",
            "colorTopicName": "/uav1/camera/color/image_raw",
            "colorCameraInfoTopicName": "/uav1/camera/color/camera_info",
            "topicName": "/uav1/camera/imu",
        }
        for tag, expected in expected_topics.items():
            values = re.findall(r"<%s>([^<]+)</%s>" % (tag, tag), jinja)
            rendered = {value.replace("{{ mavlink_id }}", "1") for value in values}
            self.assertIn(expected, rendered, msg=tag)

        plugin = _read(REALSENSE_PLUGIN)
        self.assertIn('sub_str + "/d435i_link"', plugin)
        perception = _yaml(PERCEPTION_CONFIG)
        self.assertEqual("uav1/camera_color_optical_frame",
                         perception["camera_optical_frame"])
        self.assertIn("uav1/camera_color_optical_frame",
                      EXPECTED_STATIC_TRANSFORMS)

    def test_known_legacy_duplicate_publishers_are_not_reachable(self):
        legacy_launch = _read(LEGACY_SENSOR_LAUNCH)
        legacy_bridge = _read(LEGACY_TF_BRIDGE)
        self.assertIn("tf_base_camera_$(arg uav1_id)", legacy_launch)
        self.assertIn("tf_base_camera_imu_$(arg uav1_id)", legacy_launch)
        self.assertIn("tf_mavros_base_$(arg uav1_id)", legacy_launch)
        self.assertIn("camera_color_optical_frame", legacy_bridge)
        self.assertIn("camera_depth_optical_frame", legacy_bridge)

        active = _read(RUNTIME_LAUNCH) + _read(STANDALONE_LAUNCH)
        forbidden = (
            "sitl_p450_d435i.launch",
            "sitl_outdoor_1uav_P450.launch",
            "sitl_px4_outdoor.launch",
            "uav_control_main_outdoor.launch",
            "mavros/px4.launch",
            "world_tf_bridge.py",
            "tf_base_camera_",
            "tf_mavros_base_",
        )
        for token in forbidden:
            self.assertNotIn(token, active)

    def test_mid360_jinja_freezes_mount_ray_and_message_contract(self):
        root = ET.fromstring(_read(MID360_JINJA_MODEL))
        model = root.find("./model")
        self.assertIsNotNone(model)
        livox = _one(
            [link for link in model.findall("./link")
             if link.get("name") == "livox_base"], "Livox mount link")
        _assert_float_tuple(
            self, _floats(livox.findtext("pose").split()),
            EXPECTED_LIDAR_MOUNT)
        ray = _one(
            [sensor for sensor in livox.findall("./sensor")
             if sensor.get("name") == "laser_livox"], "Livox ray sensor")
        self.assertEqual("ray", ray.get("type"))
        _assert_float_tuple(
            self, _floats(ray.findtext("pose").split()),
            EXPECTED_LIDAR_SENSOR_LOCAL)
        self.assertEqual("10", ray.findtext("update_rate"))
        plugin = _one(
            [item for item in ray.findall("./plugin")
             if item.get("filename") == "liblivox_laser_gazebo_plugins.so"],
            "Livox Gazebo plugin")
        self.assertEqual("10000", plugin.findtext("samples"))
        self.assertEqual("3", plugin.findtext("publish_pointcloud_type"))
        self.assertEqual(
            "/uav{{ mavlink_id }}/livox/lidar",
            plugin.findtext("ros_topic"))
        self.assertEqual(
            "uav{{ mavlink_id }}/lidar_link",
            plugin.findtext("frameName"))
        joint = _one(
            [item for item in model.findall("./joint")
             if item.get("name") == "mid360_joint"], "MID360 mount joint")
        self.assertEqual("fixed", joint.get("type"))
        self.assertEqual("livox_base", joint.findtext("child"))
        self.assertEqual("base_link", joint.findtext("parent"))

    def test_active_launch_surface_excludes_mapping_and_extra_lifecycle(self):
        active = (_read(RUNTIME_LAUNCH) + _read(STANDALONE_LAUNCH)).lower()
        for token in (
                "fast_lio", "octomap", "2dlidar", "2d_lidar", "mapping",
                "benchmark", "ground_aerial", "delete_model", "respawn"):
            self.assertNotIn(token, active)


class PackageAndSmokeContractTest(unittest.TestCase):
    def test_smoke_sensor_profiles_share_one_harness_with_strict_branching(self):
        source = _read(SMOKE_SCRIPT)
        mid360_entrypoint = _read(MID360_SMOKE_SCRIPT)
        self.assertTrue(os.access(str(MID360_SMOKE_SCRIPT), os.X_OK))
        self.assertIn("P450_SENSOR_PROFILE=mid360", mid360_entrypoint)
        self.assertIn('[[ $# -ne 0 ]]', mid360_entrypoint)
        self.assertIn('/usr/bin/realpath -e -- "${BASH_SOURCE[0]}"',
                      mid360_entrypoint)
        self.assertIn(
            'exec "$p450_entrypoint_dir/smoke_p450_standalone.bash"',
            mid360_entrypoint)
        self.assertNotIn("roslaunch", mid360_entrypoint)
        self.assertNotIn("with_p450_env", mid360_entrypoint)

        validation = source.index(
            "P450_SENSOR_PROFILE must be exactly d435 or mid360")
        self.assertLess(validation, source.index("p450_script_path="))
        self.assertIn('P450_SENSOR_PROFILE="$p450_sensor_profile"', source)
        self.assertIn(
            'p450_log_parent="$p450_repo_root/$p450_profile_log_root"',
            source)
        self.assertIn(
            'p450_create_private_run_directory "$p450_log_parent" '
            '"$p450_run_id"', source)
        self.assertNotIn(
            'p450_log_dir="$p450_repo_root/$p450_profile_log_root/'
            '$p450_run_id"', source)
        self.assertIn('"${p450_profile_launch_args[@]}"', source)
        self.assertIsNotNone(re.search(
            r'if \[\[ "\$p450_sensor_profile" == "d435" \]\]; then.*?'
            r'p450_check_topic_samples /uav1/camera/color/image_raw.*?'
            r'p450_check_topic_samples /uav1/camera/imu.*?else.*?'
            r'p450_mid360_topic_evidence',
            source, flags=re.DOTALL))
        self.assertIn("/uav1/livox/lidar", source)
        self.assertIn("prometheus_msgs/LivoxCustomMsg", source)
        self.assertIn("uav1/lidar_link", source)
        self.assertIn("liblivox_laser_gazebo_plugins.so", source)
        self.assertIn("libprotobuf", source)
        self.assertIn("mid360_assets.json", source)
        self.assertIn("p450_mid360_tf_contract.yaml", source)
        self.assertIn(
            "D435 profile unexpectedly loaded the Livox plugin", source)
        self.assertIn(
            "D435 profile unexpectedly advertised the Livox topic", source)

    def test_smoke_uses_a_fresh_run_local_px4_workdir(self):
        source = _read(SMOKE_SCRIPT)
        self.assertIn(
            'p450_px4_workdir="sitl_smoke_$p450_run_id"', source)
        self.assertIn(
            '[[ ! -e "$p450_px4_work_path" ]]', source)
        self.assertIn(
            'px4_workdir:="$p450_px4_workdir"', source)

    def test_smoke_shutdown_is_ordered_and_checks_launcher_status(self):
        source = _read(SMOKE_SCRIPT)
        graceful = 'kill -INT "$p450_roslaunch_pid"'
        term_group = 'kill -TERM -- -"$p450_launch_pgid"'
        kill_group = 'kill -KILL -- -"$p450_launch_pgid"'
        launcher_wait = 'wait "$p450_launch_pid" 2>/dev/null'
        self.assertIn(graceful, source)
        cleanup = re.search(
            r"^p450_cleanup\(\) \{\n(?P<body>.*?)^\}\n",
            source,
            flags=re.MULTILINE | re.DOTALL,
        )
        self.assertIsNotNone(cleanup)
        body = cleanup.group("body")
        for token in (term_group, kill_group, launcher_wait):
            self.assertEqual(1, body.count(token), token)
        self.assertLess(body.index(graceful), body.index(term_group))
        self.assertLess(body.index(term_group), body.index(kill_group))
        self.assertLess(body.index(kill_group), body.index(launcher_wait))
        self.assertIn(
            'if p450_pid_is_alive "$p450_launch_pid"; then',
            body[:body.index(launcher_wait)],
            "the shell must not wait unless the owned launcher is already "
            "stopped or a zombie",
        )
        self.assertIn(
            'p450_wait_for_group_exit 40', body,
            "TERM and KILL group-exit polls must have explicit finite bounds",
        )
        self.assertIn(
            'p450_wait_for_pid_exit "$p450_roslaunch_pid" 60', body,
            "the graceful roslaunch poll must have an explicit finite bound",
        )
        self.assertIn('p450_launch_status="$?"', source)
        self.assertNotIn(
            'wait "$p450_launch_pid" 2>/dev/null || true', source)

    def test_smoke_result_manifest_is_valid_for_pass_and_failure(self):
        source = _read(SMOKE_SCRIPT)
        guard = 'if [[ "${1:-}" == "--test-result-manifest" ]]'
        self.assertIn(
            guard,
            source,
            "offline result-manifest mode must exist before this test may "
            "execute the smoke script",
        )
        self.assertLess(source.index(guard), source.index("p450_script_path="))

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            gps = root / "SIM overlay/libgazebo_gps_plugin.so"
            groundtruth = (
                root / "SIM overlay/libgazebo_groundtruth_plugin.so")
            gps.parent.mkdir()
            gps.write_bytes(b"gps")
            groundtruth.write_bytes(b"groundtruth")
            common_evidence = {
                "checks": {
                    "gazebo_node": True,
                    "gzserver_process": True,
                    "mavros_node": True,
                    "mavros_state": True,
                    "model": True,
                    "prometheus_state": True,
                    "px4_process": True,
                    "uav_controller_node": True,
                },
                "model": {"count": 1, "name": "p450_D435i_0"},
                "tf_authorities": {
                    "map": ["/sim_world_to_map"],
                    "uav1/odom": ["/sim_localization_uav1"],
                    "uav1/base_link": ["/uav_control_main_1"],
                    "uav1/camera_link": ["/uav_control_main_1"],
                    "uav1/camera_depth_frame": [
                        "/uav1/p450_tf_camera_depth"],
                    "uav1/camera_ired1_frame": [
                        "/uav1/p450_tf_camera_ired1"],
                    "uav1/camera_ired2_frame": [
                        "/uav1/p450_tf_camera_ired2"],
                    "uav1/camera_imu_link": [
                        "/uav1/p450_tf_camera_imu"],
                    "uav1/d435i_link": [
                        "/uav1/p450_tf_camera_d435i"],
                    "uav1/camera_color_optical_frame": [
                        "/uav1/p450_tf_camera_color_optical"],
                    "uav1/camera_depth_optical_frame": [
                        "/uav1/p450_tf_depth_optical"],
                },
            }
            d435_evidence = {
                "livox_absent": True,
                "profile": "d435",
                "tf_current": True,
                "topic_gates": [
                    "/uav1/camera/color/image_raw",
                    "/uav1/camera/color/camera_info",
                    "/uav1/camera/depth/image_raw",
                    "/uav1/camera/depth/camera_info",
                    "/uav1/camera/imu",
                ],
            }
            install_prefix = REPOSITORY_ROOT / "install/p450-clean"
            asset_root = install_prefix / "share/sim_platform_assets"
            asset_manifest = json.loads(
                (REPOSITORY_ROOT / "config/mid360_assets.json").read_text(
                    encoding="utf-8"))
            asset_files = []
            for entry in asset_manifest["files"]:
                path = (asset_root / entry["target"]).resolve(strict=True)
                asset_files.append({
                    "path": str(path),
                    "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                    "target": entry["target"],
                })
            csv_path = (
                asset_root / "models/MID360/scan_mode/mid360.csv"
            ).resolve(strict=True)
            dae_path = (
                asset_root / "models/MID360/meshes/MID360.dae"
            ).resolve(strict=True)
            sdf_path = (
                asset_root / "models/MID360/MID360.sdf"
            ).resolve(strict=True)
            protobuf = root / "libprotobuf.so.17.0.0"
            protobuf.write_bytes(b"protobuf17")
            installed_contract = (
                install_prefix / "share/sim_platform_bringup/config" /
                "p450_mid360_tf_contract.yaml"
            ).resolve(strict=True)
            tf_contract = yaml.safe_load(
                installed_contract.read_text(encoding="utf-8"))
            transform = {
                "parent": tf_contract["composite"]["parent"],
                "child": tf_contract["composite"]["child"],
                "translation": tf_contract["composite"]["translation_m"],
                "rotation_xyzw": tf_contract["composite"]["rotation_xyzw"],
            }
            topic = {
                "topic": "/uav1/livox/lidar",
                "type": "prometheus_msgs/LivoxCustomMsg",
                "publishers": ["/gazebo"],
                "samples": [
                    {"stamp": {"secs": 1, "nsecs": value},
                     "frame_id": "uav1/lidar_link", "point_num": 1,
                     "points": [{"x": float(value)}]}
                    for value in (10, 20, 30)
                ],
            }
            mid360_evidence = {
                "assets": {
                    "csv": str(csv_path), "dae": str(dae_path),
                    "files": asset_files, "sdf": str(sdf_path),
                    "uris": ["model://MID360/meshes/MID360.dae"],
                },
                "csv_log": {"csv": str(csv_path)},
                "maps": {
                    "plugin": str((install_prefix / "lib" /
                        "liblivox_laser_gazebo_plugins.so").resolve(strict=True)),
                    "protobuf": [str(protobuf.resolve(strict=True))],
                    "protobuf_major": 17,
                },
                "profile": "mid360",
                "tf": {
                    "authority": "/uav_control_main_1",
                    "contract": str(installed_contract),
                    "tolerance": tf_contract["runtime_tolerance"],
                    "transform": transform,
                },
                "topic": topic,
            }
            common_path = root / "common.json"
            d435_path = root / "d435.json"
            mid360_path = root / "mid360.json"
            common_path.write_text(json.dumps(common_evidence), encoding="utf-8")
            d435_path.write_text(json.dumps(d435_evidence), encoding="utf-8")
            mid360_path.write_text(json.dumps(mid360_evidence), encoding="utf-8")
            malformed_path = root / "malformed.json"
            malformed_path.write_text("{not-json", encoding="utf-8")
            cases = (
                ("PASS", "0", "0", "0", "d435", common_path,
                 d435_path, 0, 0, False, common_evidence, d435_evidence),
                ("PASS", "0", "0", "0", "mid360", common_path,
                 mid360_path, 0, 0, False, common_evidence, mid360_evidence),
                ("FAIL", "7", "137", "1", "mid360", Path("missing"),
                 Path("missing"), 7, 137, True, None, None),
                ("FAIL", "9", "", "1", "d435", malformed_path,
                 malformed_path, 9, None, True, None, None),
            )
            for (verdict, script_status, launcher_status, escalated,
                 profile, common_fixture, sensor_fixture,
                 expected_script_status, expected_launcher_status,
                 expected_escalated, expected_common, expected_sensor) in cases:
                if profile == "mid360" and expected_common is not None:
                    expected_common = copy.deepcopy(expected_common)
                    expected_common["tf_authorities"]["uav1/lidar_link"] = [
                        "/uav_control_main_1"]
                    common_path.write_text(
                        json.dumps(expected_common), encoding="utf-8")
                elif expected_common is not None:
                    common_path.write_text(
                        json.dumps(expected_common), encoding="utf-8")
                with self.subTest(verdict=verdict,
                                  launcher_status=launcher_status):
                    manifest = root / (
                        "result-%s-%s-%s.json" %
                        (verdict.lower(), profile,
                         launcher_status or "none"))
                    result = subprocess.run(
                        [
                            str(SMOKE_SCRIPT), "--test-result-manifest",
                            str(manifest), verdict, script_status,
                            launcher_status, escalated, "run id/with spaces",
                            "sitl smoke workdir", profile, str(gps),
                            str(groundtruth), str(common_fixture),
                            str(sensor_fixture),
                        ],
                        cwd=str(REPOSITORY_ROOT),
                        env={
                            "PATH": "/usr/bin:/bin",
                            "LANG": "C.UTF-8",
                            "LC_ALL": "C.UTF-8",
                        },
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        text=True,
                        timeout=3.0,
                        check=False,
                    )
                    self.assertEqual(
                        0, result.returncode,
                        "stdout={!r} stderr={!r}".format(
                            result.stdout, result.stderr),
                    )
                    self.assertEqual(
                        {
                            "schema_version": 2,
                            "verdict": verdict,
                            "script_status": expected_script_status,
                            "launcher_status": expected_launcher_status,
                            "shutdown_escalated": expected_escalated,
                            "run_id": "run id/with spaces",
                            "px4_workdir": "sitl smoke workdir",
                            "sensor_profile": profile,
                            "plugins": {
                                "gps": str(gps.resolve()),
                                "groundtruth": str(groundtruth.resolve()),
                            },
                            "evidence": {
                                "common": expected_common,
                                "sensor": expected_sensor,
                            },
                        },
                        json.loads(manifest.read_text(encoding="utf-8")),
                    )

            for label, launcher_status, escalated in (
                    ("missing launcher", "", "0"),
                    ("failed launcher", "7", "0"),
                    ("escalated shutdown", "0", "1")):
                with self.subTest(false_pass=label):
                    manifest = root / (label.replace(" ", "-") + ".json")
                    result = subprocess.run(
                        [
                            str(SMOKE_SCRIPT), "--test-result-manifest",
                            str(manifest), "PASS", "0", launcher_status,
                            escalated, "run", "work", "d435", str(gps),
                            str(groundtruth), str(common_path), str(d435_path),
                        ],
                        cwd=str(REPOSITORY_ROOT), env={
                            "PATH": "/usr/bin:/bin", "LANG": "C.UTF-8",
                            "LC_ALL": "C.UTF-8"}, stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE, text=True, check=False)
                    self.assertNotEqual(0, result.returncode)
                    self.assertFalse(manifest.exists())

            nested_mutations = (
                ("maps plugin missing",
                 lambda value: value["maps"].pop("plugin")),
                ("maps protobuf path missing",
                 lambda value: value["maps"].pop("protobuf")),
                ("asset digest tampered",
                 lambda value: value["assets"]["files"][0].update(
                     sha256="0" * 64)),
                ("asset csv cross link",
                 lambda value: value["csv_log"].update(csv=str(dae_path))),
                ("topic type tampered",
                 lambda value: value["topic"].update(type="x/Msg")),
                ("topic sample tampered",
                 lambda value: value["topic"]["samples"][1].update(
                     frame_id="/uav1/lidar_link")),
                ("tf contract tampered",
                 lambda value: value["tf"].update(contract=str(csv_path))),
                ("tf tolerance bool",
                 lambda value: value["tf"]["tolerance"].update(
                     translation_m=True)),
                ("tf transform tampered",
                 lambda value: value["tf"]["transform"]["translation"].__setitem__(
                     0, value["tf"]["transform"]["translation"][0] + 1.0)),
            )
            strict_mid_common = copy.deepcopy(common_evidence)
            strict_mid_common["tf_authorities"]["uav1/lidar_link"] = [
                "/uav_control_main_1"]
            common_path.write_text(
                json.dumps(strict_mid_common), encoding="utf-8")
            for label, mutate in nested_mutations:
                with self.subTest(invalid_mid360=label):
                    invalid_sensor = copy.deepcopy(mid360_evidence)
                    mutate(invalid_sensor)
                    mid360_path.write_text(
                        json.dumps(invalid_sensor), encoding="utf-8")
                    manifest = root / ("invalid-" + label.replace(" ", "-") +
                                       ".json")
                    result = subprocess.run(
                        [str(SMOKE_SCRIPT), "--test-result-manifest",
                         str(manifest), "PASS", "0", "0", "0", "run",
                         "work", "mid360", str(gps), str(groundtruth),
                         str(common_path), str(mid360_path)],
                        cwd=str(REPOSITORY_ROOT), env={
                            "PATH": "/usr/bin:/bin", "LANG": "C.UTF-8",
                            "LC_ALL": "C.UTF-8"}, stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE, text=True, check=False)
                    self.assertNotEqual(0, result.returncode)
                    self.assertFalse(manifest.exists())

            tolerant_sensor = copy.deepcopy(mid360_evidence)
            tolerant_sensor["tf"]["transform"]["translation"][0] += 5e-7
            tolerant_sensor["tf"]["transform"]["rotation_xyzw"][1] -= 5e-7
            mid360_path.write_text(
                json.dumps(tolerant_sensor), encoding="utf-8")
            tolerant_manifest = root / "float32-tolerant-pass.json"
            tolerant_result = subprocess.run(
                [str(SMOKE_SCRIPT), "--test-result-manifest",
                 str(tolerant_manifest), "PASS", "0", "0", "0", "run",
                 "work", "mid360", str(gps), str(groundtruth),
                 str(common_path), str(mid360_path)],
                cwd=str(REPOSITORY_ROOT), env={
                    "PATH": "/usr/bin:/bin", "LANG": "C.UTF-8",
                    "LC_ALL": "C.UTF-8"}, stdout=subprocess.PIPE,
                stderr=subprocess.PIPE, text=True, check=False)
            self.assertEqual(0, tolerant_result.returncode,
                             tolerant_result.stderr)
            self.assertEqual(
                tolerant_sensor["tf"]["transform"],
                json.loads(tolerant_manifest.read_text(encoding="utf-8"))
                ["evidence"]["sensor"]["tf"]["transform"])

            numeric_mutations = (
                ("translation beyond tolerance", lambda value:
                 value["tf"]["transform"]["translation"].__setitem__(
                     0, value["tf"]["transform"]["translation"][0] + 2e-6)),
                ("quaternion beyond tolerance", lambda value:
                 value["tf"]["transform"]["rotation_xyzw"].__setitem__(
                     1, value["tf"]["transform"]["rotation_xyzw"][1] + 2e-6)),
                ("translation bool", lambda value:
                 value["tf"]["transform"]["translation"].__setitem__(1, True)),
                ("quaternion nan", lambda value:
                 value["tf"]["transform"]["rotation_xyzw"].__setitem__(
                     1, float("nan"))),
            )
            for label, mutate in numeric_mutations:
                with self.subTest(invalid_manifest_tf=label):
                    invalid_sensor = copy.deepcopy(mid360_evidence)
                    mutate(invalid_sensor)
                    mid360_path.write_text(
                        json.dumps(invalid_sensor), encoding="utf-8")
                    manifest = root / ("numeric-" + label.replace(" ", "-") +
                                       ".json")
                    result = subprocess.run(
                        [str(SMOKE_SCRIPT), "--test-result-manifest",
                         str(manifest), "PASS", "0", "0", "0", "run",
                         "work", "mid360", str(gps), str(groundtruth),
                         str(common_path), str(mid360_path)],
                        cwd=str(REPOSITORY_ROOT), env={
                            "PATH": "/usr/bin:/bin", "LANG": "C.UTF-8",
                            "LC_ALL": "C.UTF-8"}, stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE, text=True, check=False)
                    self.assertNotEqual(0, result.returncode)
                    self.assertFalse(manifest.exists())

    def test_smoke_result_manifest_rejects_false_pass_and_never_overwrites(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            gps = root / "libgazebo_gps_plugin.so"
            groundtruth = root / "libgazebo_groundtruth_plugin.so"
            gps.write_bytes(b"gps")
            groundtruth.write_bytes(b"groundtruth")
            missing = root / "missing-evidence.json"
            base = [
                str(SMOKE_SCRIPT), "--test-result-manifest", "RESULT",
                "PASS", "0", "0", "0", "run", "work", "d435",
                str(gps), str(groundtruth), str(missing), str(missing),
            ]
            result_path = root / "false-pass.json"
            command = list(base)
            command[2] = str(result_path)
            result = subprocess.run(
                command, cwd=str(REPOSITORY_ROOT), env={
                    "PATH": "/usr/bin:/bin", "LANG": "C.UTF-8",
                    "LC_ALL": "C.UTF-8"}, stdout=subprocess.PIPE,
                stderr=subprocess.PIPE, text=True, check=False)
            self.assertNotEqual(0, result.returncode)
            self.assertFalse(result_path.exists())

            fallback_path = root / "fallback-fail.json"
            fallback = list(base)
            fallback[1] = "--test-result-fallback"
            fallback[2] = str(fallback_path)
            result = subprocess.run(
                fallback, cwd=str(REPOSITORY_ROOT), env={
                    "PATH": "/usr/bin:/bin", "LANG": "C.UTF-8",
                    "LC_ALL": "C.UTF-8"}, stdout=subprocess.PIPE,
                stderr=subprocess.PIPE, text=True, check=False)
            self.assertNotEqual(0, result.returncode)
            self.assertTrue(fallback_path.is_file(), result.stderr)
            fallback_payload = json.loads(
                fallback_path.read_text(encoding="utf-8"))
            self.assertEqual(2, fallback_payload["schema_version"])
            self.assertEqual("FAIL", fallback_payload["verdict"])
            self.assertEqual(1, fallback_payload["script_status"])
            self.assertEqual(
                {"common": None, "sensor": None},
                fallback_payload["evidence"])
            self.assertEqual(
                [], list(root.glob(".fallback-fail.json.*.tmp")),
                "a rejected PASS write must not leave a temporary manifest")

            existing = root / "existing.json"
            existing.write_text('{"sentinel":true}\n', encoding="utf-8")
            command[2] = str(existing)
            command[3:6] = ["FAIL", "9", ""]
            result = subprocess.run(
                command, cwd=str(REPOSITORY_ROOT), env={
                    "PATH": "/usr/bin:/bin", "LANG": "C.UTF-8",
                    "LC_ALL": "C.UTF-8"}, stdout=subprocess.PIPE,
                stderr=subprocess.PIPE, text=True, check=False)
            self.assertNotEqual(0, result.returncode)
            self.assertEqual(
                {"sentinel": True},
                json.loads(existing.read_text(encoding="utf-8")))

            existing_fallback = list(base)
            existing_fallback[1] = "--test-result-fallback"
            existing_fallback[2] = str(existing)
            result = subprocess.run(
                existing_fallback, cwd=str(REPOSITORY_ROOT), env={
                    "PATH": "/usr/bin:/bin", "LANG": "C.UTF-8",
                    "LC_ALL": "C.UTF-8"}, stdout=subprocess.PIPE,
                stderr=subprocess.PIPE, text=True, check=False)
            self.assertNotEqual(0, result.returncode)
            self.assertEqual(
                {"sentinel": True},
                json.loads(existing.read_text(encoding="utf-8")))

            invalid_profile = list(base)
            invalid_profile[2] = str(root / "invalid-profile.json")
            invalid_profile[9] = "livox"
            result = subprocess.run(
                invalid_profile, cwd=str(REPOSITORY_ROOT), env={
                    "PATH": "/usr/bin:/bin", "LANG": "C.UTF-8",
                    "LC_ALL": "C.UTF-8"}, stdout=subprocess.PIPE,
                stderr=subprocess.PIPE, text=True, check=False)
            self.assertNotEqual(0, result.returncode)

    def test_package_installs_runtime_assets_and_declares_dependencies(self):
        cmake = _read(PACKAGE_ROOT / "CMakeLists.txt")
        self.assertIn("cmake_minimum_required(VERSION 3.0.2)", cmake)
        self.assertRegex(
            cmake,
            r"install\s*\(\s*DIRECTORY\s+launch\s+config\s+"
            r"DESTINATION\s+\$\{CATKIN_PACKAGE_SHARE_DESTINATION\}\s*\)",
        )
        self.assertIn(
            "catkin_add_nosetests(test/test_p450_launch_contract.py)", cmake)

        manifest = _xml(PACKAGE_ROOT / "package.xml")
        dependencies = {item.text.strip() for item in manifest.findall("exec_depend")}
        self.assertTrue({
            "gazebo_ros",
            "mavros",
            "prometheus_gazebo",
            "prometheus_uav_control",
            "realsense_ros_gazebo",
            "gazebo_plugins",
            "roslaunch",
            "tf2_ros",
            "python3-jinja2",
            "python3-numpy",
            "python3-rospkg",
            "python3-yaml",
            "livox_laser_gazebo_plugins",
            "sim_platform_assets",
        }.issubset(dependencies))
        dependency_text = "\n".join(dependencies).lower()
        for token in ("fast_lio", "octomap", "mapping", "benchmark"):
            self.assertNotIn(token, dependency_text)

    def test_smoke_script_is_isolated_bounded_and_checks_live_state(self):
        source = _read(SMOKE_SCRIPT)
        required_tokens = (
            "P450_SMOKE_INNER",
            "scripts/with_p450_env.bash",
            "ROS_MASTER_URI",
            "GAZEBO_MASTER_URI",
            "127.0.0.1",
            "4560",
            "14540",
            "14560",
            "14580",
            "/usr/bin/ss",
            "/usr/bin/setsid",
            "/usr/bin/timeout",
            "trap",
            "kill -TERM -- -",
            "logs/",
            "p450_standalone.launch",
            "gui:=false",
            "/gazebo/get_world_properties",
            "p450_D435i_0",
            "/uav1/mavros",
            "/uav1/sitl_1",
            "/uav_control_main_1",
            "/uav1/mavros/state",
            "/uav1/prometheus/state",
            "/uav1/camera/color/image_raw",
            "/uav1/camera/color/camera_info",
            "/uav1/camera/depth/image_raw",
            "/uav1/camera/depth/camera_info",
            "/uav1/camera/imu",
            "/tf",
            "/tf_static",
            "connected",
            "armed",
            "odom_valid",
            "frame_id",
            "stamp",
        )
        for token in required_tokens:
            self.assertIn(token, source)
        self.assertNotRegex(source, r"\b(?:pkill|killall)\b")
        self.assertNotIn("rosnode kill", source)
        self.assertNotIn("/mavros/cmd/arming", source)
        self.assertNotRegex(source, r"\b(?:takeoff|OFFBOARD)\b")

    def test_smoke_process_probe_follows_launch_ppid_tree(self):
        source = _read(SMOKE_SCRIPT)
        guard = 'if [[ "${1:-}" == "--test-process-tree" ]]'
        self.assertIn(
            guard,
            source,
            "offline process-tree mode must exist before this test may execute "
            "the smoke script",
        )
        self.assertLess(
            source.index(guard),
            source.index("p450_script_path="),
            "offline process-tree mode must return before wrapper/preflight/launch",
        )

        process_table = """\
100 1 100 Ss /usr/bin/setsid --wait root-self
101 100 100 S /usr/bin/timeout 300s roslaunch
102 101 100 Sl /usr/bin/python3 /opt/ros/noetic/bin/roslaunch
201 102 201 Ssl /usr/bin/gzserver --verbose
202 102 202 Ssl /opt/px4/build/amovlab_sitl_default/bin/px4 -d
203 102 203 Z [zombie-only] <defunct>
300 1 300 Ssl /usr/bin/ambient-only
501 9999 501 Ssl /usr/bin/missing-chain
600 601 600 S /usr/bin/cycle-only
601 600 601 S /bin/sh
"""
        cases = (
            ("100", "gzserver", 0),
            ("100", "/opt/px4/build/amovlab_sitl_default/bin/px4", 0),
            ("100", "zombie-only", 1),
            ("100", "ambient-only", 1),
            ("100", "missing-chain", 1),
            ("100", "cycle-only", 1),
            ("100", "root-self", 1),
            ("0", "gzserver", 64),
            ("not-a-pid", "gzserver", 64),
            ("1", "gzserver", 64),
            ("100", "", 64),
        )
        for root_pid, needle, expected_status in cases:
            with self.subTest(root_pid=root_pid, needle=needle):
                result = subprocess.run(
                    [str(SMOKE_SCRIPT), "--test-process-tree",
                     root_pid, needle],
                    cwd=str(REPOSITORY_ROOT),
                    env={
                        "PATH": "/usr/bin:/bin",
                        "LANG": "C.UTF-8",
                        "LC_ALL": "C.UTF-8",
                    },
                    input=process_table,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    timeout=3.0,
                    check=False,
                )
                self.assertEqual(
                    expected_status,
                    result.returncode,
                    "stdout={!r} stderr={!r}".format(
                        result.stdout, result.stderr),
                )

    def test_smoke_log_probe_rejects_fatal_runtime_events(self):
        source = _read(SMOKE_SCRIPT)
        guard = 'if [[ "${1:-}" == "--test-launch-log" ]]'
        self.assertIn(
            guard,
            source,
            "offline launch-log mode must exist before this test may execute "
            "the smoke script",
        )
        self.assertLess(source.index(guard), source.index("p450_script_path="))

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            cases = (
                (
                    "clean.log",
                    "INFO [px4] Startup script returned successfully\n"
                    "process has finished cleanly\n"
                    "CSV statistics: 10000 rows loaded\n",
                    0,
                ),
                ("startup-missing.log", "process has finished cleanly\n", 1),
                (
                    "startup-twice.log",
                    "Startup script returned successfully\n"
                    "Startup script returned successfully\n",
                    1,
                ),
                (
                    "startup-failed.log",
                    "ERROR [px4] Startup script returned with return value: 2\n",
                    1,
                ),
                ("segv.log", "Segmentation fault (core dumped)\n", 1),
                (
                    "assert.log",
                    "gzserver: model.cc:42: Assertion `px != 0' failed.\n",
                    1,
                ),
                ("reboot.log", "[uav_controller_uav1] Reboot PX4!\n", 1),
                ("died.log", "[gazebo-2] process has died [pid 42]\n", 1),
                (
                    "livox-csv-missing.log",
                    "Startup script returned successfully\n"
                    "cannot get csv file!/tmp/missing.csvwill return !\n",
                    1,
                ),
                (
                    "livox-frame-empty.log",
                    "Startup script returned successfully\n"
                    "Livox frameName must not be empty\n",
                    1,
                ),
                (
                    "livox-plugin-load.log",
                    "Startup script returned successfully\n"
                    "Failed to load plugin liblivox_laser_gazebo_plugins.so\n",
                    1,
                ),
                ("missing.log", None, 66),
            )
            for filename, contents, expected_status in cases:
                with self.subTest(filename=filename):
                    launch_log = root / filename
                    if contents is not None:
                        launch_log.write_text(contents, encoding="utf-8")
                    result = subprocess.run(
                        [str(SMOKE_SCRIPT), "--test-launch-log",
                         str(launch_log)],
                        cwd=str(REPOSITORY_ROOT),
                        env={
                            "PATH": "/usr/bin:/bin",
                            "LANG": "C.UTF-8",
                            "LC_ALL": "C.UTF-8",
                        },
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        text=True,
                        timeout=3.0,
                        check=False,
                    )
                    self.assertEqual(
                        expected_status,
                        result.returncode,
                        "stdout={!r} stderr={!r}".format(
                            result.stdout, result.stderr),
                    )

    def test_smoke_tf_authority_probe_rejects_any_duplicate_child(self):
        source = _read(SMOKE_SCRIPT)
        guard = 'if [[ "${1:-}" == "--test-tf-authorities" ]]'
        self.assertIn(guard, source)
        self.assertLess(source.index(guard), source.index("p450_script_path="))

        expected = {
            "map": ["/sim_world_to_map"],
            "uav1/odom": ["/sim_localization_uav1"],
            "uav1/base_link": ["/uav_control_main_1"],
            "uav1/camera_link": ["/uav_control_main_1"],
            "uav1/camera_depth_frame": ["/uav1/p450_tf_camera_depth"],
            "uav1/camera_ired1_frame": ["/uav1/p450_tf_camera_ired1"],
            "uav1/camera_ired2_frame": ["/uav1/p450_tf_camera_ired2"],
            "uav1/camera_imu_link": ["/uav1/p450_tf_camera_imu"],
            "uav1/d435i_link": ["/uav1/p450_tf_camera_d435i"],
            "uav1/camera_color_optical_frame": [
                "/uav1/p450_tf_camera_color_optical"],
            "uav1/camera_depth_optical_frame": [
                "/uav1/p450_tf_depth_optical"],
            "uav1/lidar_link": ["/one_lidar_authority"],
        }
        duplicate = copy.deepcopy(expected)
        duplicate["uav1/lidar_link"].append("/second_lidar_authority")

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            cases = (
                ("valid.json", expected, 0),
                ("duplicate.json", duplicate, 1),
                ("malformed.json", [], 65),
            )
            for filename, payload, expected_status in cases:
                with self.subTest(filename=filename):
                    path = root / filename
                    path.write_text(json.dumps(payload), encoding="utf-8")
                    result = subprocess.run(
                        [str(SMOKE_SCRIPT), "--test-tf-authorities", str(path)],
                        cwd=str(REPOSITORY_ROOT),
                        env={
                            "PATH": "/usr/bin:/bin",
                            "LANG": "C.UTF-8",
                            "LC_ALL": "C.UTF-8",
                        },
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        text=True,
                        timeout=3.0,
                        check=False,
                    )
                    self.assertEqual(
                        expected_status,
                        result.returncode,
                        "stdout={!r} stderr={!r}".format(
                            result.stdout, result.stderr),
                    )

    def test_smoke_plugin_maps_probe_requires_only_exact_sim_plugin(self):
        source = _read(SMOKE_SCRIPT)
        guard = 'if [[ "${1:-}" == "--test-plugin-maps" ]]'
        self.assertIn(
            guard,
            source,
            "offline plugin-map mode must exist before this test may execute "
            "the smoke script",
        )
        self.assertLess(source.index(guard), source.index("p450_script_path="))
        self.assertIn(
            'p450_gzserver_executable="$(/usr/bin/realpath -e -- '
            '/usr/bin/gzserver)"',
            source,
        )

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            local_plugin = (
                root / "local overlay/libgazebo_groundtruth_plugin.so")
            external_plugin = (
                root / "external px4/libgazebo_groundtruth_plugin.so")
            third_plugin = (
                root / "third copy/libgazebo_groundtruth_plugin.so")
            local_plugin.parent.mkdir()
            external_plugin.parent.mkdir()
            third_plugin.parent.mkdir()
            local_plugin.write_bytes(b"local")
            external_plugin.write_bytes(b"external")
            third_plugin.write_bytes(b"third")
            prefix = "7f000000-7f001000 r-xp 00000000 08:01 42 "
            proc_path = lambda path: str(path).replace(" ", r"\040")
            cases = (
                ("local.maps", prefix + proc_path(local_plugin) + "\n", 0),
                (
                    "local-plus-deleted.maps",
                    prefix + proc_path(local_plugin) + "\n" +
                    prefix + proc_path(local_plugin) + " (deleted)\n",
                    1,
                ),
                (
                    "local-deleted-only.maps",
                    prefix + proc_path(local_plugin) + " (deleted)\n",
                    1,
                ),
                (
                    "both.maps",
                    prefix + proc_path(local_plugin) + "\n" +
                    prefix + proc_path(external_plugin) + "\n",
                    1,
                ),
                (
                    "third.maps",
                    prefix + proc_path(local_plugin) + "\n" +
                    prefix + proc_path(third_plugin) + "\n",
                    1,
                ),
                (
                    "external.maps",
                    prefix + proc_path(external_plugin) + "\n",
                    1,
                ),
                ("missing.maps", None, 66),
            )
            for filename, contents, expected_status in cases:
                with self.subTest(filename=filename):
                    maps = root / filename
                    if contents is not None:
                        maps.write_text(contents, encoding="utf-8")
                    result = subprocess.run(
                        [
                            str(SMOKE_SCRIPT), "--test-plugin-maps",
                            str(local_plugin), str(external_plugin), str(maps),
                        ],
                        cwd=str(REPOSITORY_ROOT),
                        env={
                            "PATH": "/usr/bin:/bin",
                            "LANG": "C.UTF-8",
                            "LC_ALL": "C.UTF-8",
                        },
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        text=True,
                        timeout=3.0,
                        check=False,
                    )
                    self.assertEqual(
                        expected_status,
                        result.returncode,
                        "stdout={!r} stderr={!r}".format(
                            result.stdout, result.stderr),
                    )

    def test_smoke_waits_for_complete_profile_plugin_mappings(self):
        source = _read(SMOKE_SCRIPT)
        readiness_mode = (
            'if [[ "${1:-}" == "--test-plugin-map-readiness" ]]')
        self.assertIn(readiness_mode, source)
        self.assertIn(
            'p450_wait_until "complete Gazebo plugin mappings"', source)
        self.assertLess(
            source.index(
                'p450_wait_until "complete Gazebo plugin mappings"'),
            source.index('p450_check_state /uav1/mavros/state'),
        )

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            local = root / "SIM overlay"
            external = root / "external PX4"
            installed = root / "installed Livox"
            system = root / "system protobuf"
            for directory in (local, external, installed, system):
                directory.mkdir()

            gps = local / "libgazebo_gps_plugin.so"
            groundtruth = local / "libgazebo_groundtruth_plugin.so"
            external_gps = external / gps.name
            external_groundtruth = external / groundtruth.name
            livox = installed / "liblivox_laser_gazebo_plugins.so"
            protobuf = system / "libprotobuf.so.17.0.0"
            for artifact in (
                    gps, groundtruth, external_gps, external_groundtruth,
                    livox, protobuf):
                artifact.write_bytes(b"fixture")

            prefix = "7f000000-7f001000 r-xp 00000000 08:01 42 "

            def mapping(path):
                return prefix + str(path).replace(" ", r"\040") + "\n"

            early_maps = root / "early.maps"
            early_maps.write_text(
                mapping(gps) + mapping(livox) + mapping(protobuf),
                encoding="utf-8",
            )
            ready_maps = root / "ready.maps"
            ready_maps.write_text(
                mapping(gps) + mapping(groundtruth) +
                mapping(livox) + mapping(protobuf),
                encoding="utf-8",
            )

            def probe(maps, evidence, profile="mid360"):
                return subprocess.run(
                    [
                        str(SMOKE_SCRIPT), "--test-plugin-map-readiness",
                        profile, str(gps), str(external_gps),
                        str(groundtruth), str(external_groundtruth),
                        str(livox), str(maps), str(evidence),
                    ],
                    cwd=str(REPOSITORY_ROOT),
                    env={
                        "PATH": "/usr/bin:/bin",
                        "LANG": "C.UTF-8",
                        "LC_ALL": "C.UTF-8",
                    },
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    timeout=3.0,
                    check=False,
                )

            early_evidence = root / "early-evidence.json"
            early = probe(early_maps, early_evidence)
            self.assertNotEqual(0, early.returncode)
            self.assertFalse(early_evidence.exists())

            ready_evidence = root / "ready-evidence.json"
            ready = probe(ready_maps, ready_evidence)
            self.assertEqual(
                0, ready.returncode,
                "stdout={!r} stderr={!r}".format(
                    ready.stdout, ready.stderr),
            )
            self.assertEqual(
                {
                    "plugin": str(livox.resolve(strict=True)),
                    "protobuf": [str(protobuf.resolve(strict=True))],
                    "protobuf_major": 17,
                },
                json.loads(ready_evidence.read_text(encoding="utf-8")),
            )

            malformed_maps = root / "malformed.maps"
            malformed_maps.write_text(
                mapping(gps) + mapping(groundtruth) + mapping(livox),
                encoding="utf-8",
            )
            malformed_evidence = root / "malformed-evidence.json"
            malformed = probe(malformed_maps, malformed_evidence)
            self.assertEqual(65, malformed.returncode)
            self.assertFalse(malformed_evidence.exists())

            evidence_directory = root / "evidence-directory"
            evidence_directory.mkdir()
            directory_result = probe(ready_maps, evidence_directory)
            self.assertNotEqual(0, directory_result.returncode)
            self.assertEqual([], list(evidence_directory.iterdir()))

            d435_evidence = root / "d435-evidence.json"
            d435_ready = root / "d435-ready.maps"
            d435_ready.write_text(
                mapping(gps) + mapping(groundtruth), encoding="utf-8")
            d435 = probe(d435_ready, d435_evidence, profile="d435")
            self.assertEqual(0, d435.returncode, d435.stderr)
            self.assertFalse(d435_evidence.exists())
            d435_with_livox = probe(
                ready_maps, d435_evidence, profile="d435")
            self.assertNotEqual(0, d435_with_livox.returncode)
            self.assertFalse(d435_evidence.exists())

    def test_smoke_refreshes_read_only_proc_maps_snapshot_atomically(self):
        source = _read(SMOKE_SCRIPT)
        guard = 'if [[ "${1:-}" == "--test-refresh-maps-snapshot" ]]'
        self.assertIn(guard, source)
        self.assertLess(source.index(guard), source.index("p450_script_path="))

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            live_maps = root / "proc-maps"
            snapshot = root / "gzserver.maps"
            snapshot.write_text("stale\n", encoding="utf-8")
            snapshot.chmod(0o444)

            def refresh(expected):
                live_maps.write_text(expected, encoding="utf-8")
                result = subprocess.run(
                    [
                        str(SMOKE_SCRIPT), "--test-refresh-maps-snapshot",
                        str(live_maps), str(snapshot),
                    ],
                    cwd=str(REPOSITORY_ROOT),
                    env={
                        "PATH": "/usr/bin:/bin",
                        "LANG": "C.UTF-8",
                        "LC_ALL": "C.UTF-8",
                    },
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    timeout=3.0,
                    check=False,
                )
                self.assertEqual(
                    0, result.returncode,
                    "stdout={!r} stderr={!r}".format(
                        result.stdout, result.stderr),
                )
                self.assertEqual(expected, snapshot.read_text(encoding="utf-8"))
                self.assertTrue(snapshot.stat().st_mode & stat.S_IWUSR)
                self.assertEqual([], list(root.glob(".gzserver.maps.tmp.*")))

            refresh("early\n")
            snapshot.chmod(0o444)
            refresh("ready\n")

            directory_parent = root / "directory-target-parent"
            directory_parent.mkdir()
            directory_target = directory_parent / "gzserver.maps"
            directory_target.mkdir()
            rejected = subprocess.run(
                [
                    str(SMOKE_SCRIPT), "--test-refresh-maps-snapshot",
                    str(live_maps), str(directory_target),
                ],
                cwd=str(REPOSITORY_ROOT),
                env={
                    "PATH": "/usr/bin:/bin",
                    "LANG": "C.UTF-8",
                    "LC_ALL": "C.UTF-8",
                },
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=3.0,
                check=False,
            )
            self.assertNotEqual(0, rejected.returncode)
            self.assertEqual([], list(directory_target.iterdir()))
            self.assertEqual(
                [], list(directory_parent.glob(".gzserver.maps.tmp.*")))

    def test_smoke_creates_private_run_leaf_exclusively(self):
        source = _read(SMOKE_SCRIPT)
        guard = 'if [[ "${1:-}" == "--test-private-run-directory" ]]'
        self.assertIn(guard, source)
        self.assertIn("umask 077", source)
        self.assertLess(source.index(guard), source.index("p450_script_path="))

        with tempfile.TemporaryDirectory() as temporary:
            parent = Path(temporary) / "profile logs"
            parent.mkdir(mode=0o775)
            run_id = "20260901T001122Z-12345"
            command = [
                str(SMOKE_SCRIPT), "--test-private-run-directory",
                str(parent), run_id,
            ]
            environment = {
                "PATH": "/usr/bin:/bin",
                "LANG": "C.UTF-8",
                "LC_ALL": "C.UTF-8",
            }

            created = subprocess.run(
                command,
                cwd=str(REPOSITORY_ROOT),
                env=environment,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=3.0,
                check=False,
            )
            self.assertEqual(0, created.returncode, created.stderr)
            leaf = parent / run_id
            self.assertEqual(str(leaf) + "\n", created.stdout)
            self.assertTrue(leaf.is_dir())
            self.assertFalse(leaf.is_symlink())
            self.assertEqual(0o700, stat.S_IMODE(leaf.stat().st_mode))

            duplicate = subprocess.run(
                command,
                cwd=str(REPOSITORY_ROOT),
                env=environment,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=3.0,
                check=False,
            )
            self.assertNotEqual(0, duplicate.returncode)
            self.assertTrue(leaf.is_dir())

            symlink_id = "20260901T001123Z-12345"
            outside = Path(temporary) / "outside"
            outside.mkdir()
            (parent / symlink_id).symlink_to(outside, target_is_directory=True)
            symlink = subprocess.run(
                [
                    str(SMOKE_SCRIPT), "--test-private-run-directory",
                    str(parent), symlink_id,
                ],
                cwd=str(REPOSITORY_ROOT),
                env=environment,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=3.0,
                check=False,
            )
            self.assertNotEqual(0, symlink.returncode)
            self.assertEqual([], list(outside.iterdir()))

    def test_smoke_render_probe_requires_local_x_capability(self):
        source = _read(SMOKE_SCRIPT)
        guard = 'if [[ "${1:-}" == "--test-render-capability" ]]'
        self.assertIn(
            guard,
            source,
            "offline render mode must exist before this test may execute "
            "the smoke script",
        )
        self.assertLess(source.index(guard), source.index("p450_script_path="))

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            socket_root = root / "x11 sockets"
            socket_root.mkdir()
            xauthority = root / "authority real"
            xauthority.write_text("test cookie\n", encoding="utf-8")
            authority_link = root / "authority link"
            authority_link.symlink_to(xauthority)
            display_socket, display_peer = socket.socketpair()
            (socket_root / "X91").symlink_to(
                "/proc/self/fd/%d" % display_socket.fileno())
            try:
                cases = (
                    (":91.0", authority_link, 0),
                    ("remote.example:0", authority_link, 64),
                    (":91", root / "missing authority", 66),
                    (":92", authority_link, 69),
                )
                for display, authority, expected_status in cases:
                    with self.subTest(display=display, authority=authority):
                        result = subprocess.run(
                            [
                                str(SMOKE_SCRIPT),
                                "--test-render-capability",
                                display,
                                str(authority),
                                str(socket_root),
                            ],
                            cwd=str(REPOSITORY_ROOT),
                            env={
                                "PATH": "/usr/bin:/bin",
                                "LANG": "C.UTF-8",
                                "LC_ALL": "C.UTF-8",
                            },
                            stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE,
                            text=True,
                            timeout=3.0,
                            check=False,
                            pass_fds=(display_socket.fileno(),),
                        )
                        self.assertEqual(
                            expected_status,
                            result.returncode,
                            "stdout={!r} stderr={!r}".format(
                                result.stdout, result.stderr),
                        )
                        if expected_status == 0:
                            self.assertEqual(
                                "DISPLAY=:91.0\nXAUTHORITY=%s\n" %
                                xauthority.resolve(),
                                result.stdout,
                            )
            finally:
                display_socket.close()
                display_peer.close()


if __name__ == "__main__":
    unittest.main()
