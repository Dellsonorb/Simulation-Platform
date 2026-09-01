#!/usr/bin/env python3
import argparse
import copy
import hashlib
import json
import math
import os
import re
import stat
import statistics
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
RUN_ROOT = REPOSITORY_ROOT / "logs/bunker_standalone"
INSTALL_ROOT = REPOSITORY_ROOT / "install/p450-clean"
RUNTIME_URDF_SHA256 = (
    "2b58856bed616a9e7402f7ddf4be4336f669271ca3989fdcf1d50f6939270832")
FROZEN_XACRO_SHA256 = (
    "41e6d862c476264dfb8d150c0f8e1780da2451867908ff6e38300813a2b54f74")
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
RUN_ID_RE = re.compile(r"^[0-9]{8}T[0-9]{6}Z-[1-9][0-9]*$")
UTC_RE = re.compile(
    r"^[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9]{2}:"
    r"[0-9]{2}:[0-9]{2}(?:\.[0-9]+)?Z$")
URI_RE = re.compile(r"^http://127\.0\.0\.1:([1-9][0-9]{0,4})/?$")

VECTOR3_KEYS = frozenset({"x", "y", "z"})
QUATERNION_KEYS = frozenset({"x", "y", "z", "w"})
ODOMETRY_SAMPLE_KEYS = frozenset({
    "stamp", "frame_id", "child_frame_id", "position", "orientation",
    "linear_twist", "angular_twist",
})
SCAN_SAMPLE_KEYS = frozenset({
    "stamp", "frame_id", "ranges", "intensities", "angle_min",
    "angle_max", "angle_increment", "range_min", "range_max",
    "time_increment", "scan_time",
})
SCAN_CONTRACT = {
    "frame_id": "ground/lidar_2d_link",
    "range_count": 720,
    "intensity_count": 720,
    "angle_min": -math.pi,
    "angle_max": math.pi,
    "angle_increment": 2.0 * math.pi / 719.0,
    "range_min": 0.12,
    "range_max": 8.0,
    "time_increment": 0.0,
    "scan_time": 0.0,
}

GUARD_RECORD_KEYS = frozenset({
    "case", "input_tokens", "output", "input_stamp", "output_stamp",
    "latency",
})
GUARD_CASE_TOKENS = {
    "over_limit": ("0.75", "0", "0", "0", "0", "2.0"),
    "nonplanar": ("0", "0.1", "0", "0", "0", "0"),
    "nonfinite": ("nan", "0", "0", "0", "0", "0"),
}
GUARD_EXPECTED_OUTPUTS = {
    "over_limit": (0.5, 0.0, 0.0, 0.0, 0.0, 1.0),
    "nonplanar": (0.0,) * 6,
    "nonfinite": (0.0,) * 6,
}

FORWARD_LIMITS = {
    "forward": (0.50, 0.70), "abs_lateral_max": 0.10,
    "model_odom_xy_max": 0.03, "model_odom_yaw_max": 0.04,
}
ROTATION_LIMITS = {
    "yaw": (0.85, 1.15), "planar_drift_max": 0.08,
    "model_odom_xy_max": 0.03, "model_odom_yaw_max": 0.04,
}
STOP_LIMITS = {
    "minimum_clock_coast": 0.5,
    "model_linear_speed_max": 0.01,
    "model_angular_speed_max": 0.01,
    "odom_linear_speed_max": 0.01,
    "odom_angular_speed_max": 0.01,
    "stationary_window": 1.0,
    "stationary_position_drift_max": 0.01,
    "stationary_yaw_drift_max": 0.01,
}
MOTION_PHASE_KEYS = frozenset({
    "kind", "command", "baseline", "final", "watchdog", "stationary",
    "metrics",
})
COMMAND_KEYS = frozenset({
    "linear_x", "angular_z", "rate_hz", "target_duration",
    "start_stamp", "last_publish_stamp", "end_stamp", "published_count",
    "actual_duration", "minimum_period", "median_period", "maximum_period",
})
WATCHDOG_KEYS = frozenset({
    "last_command_stamp", "stop_observed_stamp", "clock_coast",
    "time_to_stop", "model_linear_speed", "model_angular_speed",
    "odom_linear_speed", "odom_angular_speed",
})
STATIONARY_KEYS = frozenset({
    "start_stamp", "end_stamp", "duration", "position_drift", "yaw_drift",
})
POSE_KEYS = frozenset({"model", "odom"})
MODEL_POSE_KEYS = frozenset({"x", "y", "z", "roll", "pitch", "yaw"})
ODOM_POSE_KEYS = frozenset({"x", "y", "yaw"})
FORWARD_METRIC_KEYS = frozenset({
    "model_forward", "model_lateral", "odom_forward", "odom_lateral",
    "model_odom_xy", "model_odom_yaw", "height_change", "roll", "pitch",
})
ROTATION_METRIC_KEYS = frozenset({
    "model_yaw", "model_planar_drift", "odom_yaw", "model_odom_xy",
    "model_odom_yaw", "height_change", "roll", "pitch",
})

EXPECTED_GRAPH = {
    "/ground/cmd_vel": {
        "type": "geometry_msgs/Twist",
        "publishers": frozenset(),
        "subscribers": frozenset({"/ground/velocity_guard"}),
    },
    "/ground/cmd_vel_safe": {
        "type": "geometry_msgs/Twist",
        "publishers": frozenset({"/ground/velocity_guard"}),
        "subscribers": frozenset({"/gazebo"}),
    },
    "/ground/odom": {
        "type": "nav_msgs/Odometry",
        "publishers": frozenset({"/gazebo"}),
        "subscribers": frozenset(),
    },
    "/ground/scan": {
        "type": "sensor_msgs/LaserScan",
        "publishers": frozenset({"/gazebo"}),
        "subscribers": frozenset(),
    },
}
EXPECTED_FIXED_CHILDREN = (
    "ground/lidar_2d_link",
    "ground/wheel1_Link", "ground/wheel1_1_Link",
    "ground/wheel1_2_Link", "ground/wheel1_3_Link",
    "ground/wheel2_Link", "ground/wheel2_1_Link",
    "ground/wheel2_2_Link", "ground/wheel2_3_Link",
    "ground/wheel3_Link", "ground/wheel3_1_Link",
    "ground/wheel3_2_Link", "ground/wheel3_3_Link",
    "ground/wheel4_Link", "ground/wheel4_1_Link",
    "ground/wheel4_2_Link", "ground/wheel4_3_Link",
)
EXPECTED_TF_AUTHORITIES = {
    ("world", "ground/odom", "tf_static"): "/ground/world_to_odom",
    ("ground/odom", "ground/base_link", "tf"): "/gazebo",
}
EXPECTED_TF_AUTHORITIES.update({
    ("ground/base_link", child, "tf_static"):
        "/ground/robot_state_publisher"
    for child in EXPECTED_FIXED_CHILDREN
})
TF_EDGE_KEYS = frozenset({
    "parent", "child", "channel", "authority", "stamp",
    "translation", "rotation",
})

PLUGIN_PATHS = (
    "/opt/ros/noetic/lib/libgazebo_ros_planar_move.so",
    "/opt/ros/noetic/lib/libgazebo_ros_laser.so",
    "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins/libRayPlugin.so",
)
PLUGIN_RECORD_KEYS = frozenset({"basename", "path"})
COLLISION_RECORD_KEYS = frozenset({
    "name", "link", "geometry", "origin", "size",
})
COLLISION_KEYS = frozenset({
    "model", "link", "collisions", "mesh_collision_count",
    "baseline_height", "final_height", "roll", "pitch",
    "raw_info_path", "raw_info_sha256",
})
COLLISION_ORIGIN = (0.018058912, 0.001357451, -0.160420741)
COLLISION_SIZE = (1.026335219, 0.782744936, 0.395154782)

PROBE_SCHEMA_VERSION = 1
LAUNCHER_SCHEMA_VERSION = 1
RESULT_SCHEMA_VERSION = 2
PROBE_KEYS = frozenset({
    "schema_version", "status", "fatal_errors", "run", "environment",
    "provenance", "processes", "graph", "samples", "guard", "motion",
    "tf", "collision",
})
RESULT_KEYS = PROBE_KEYS | frozenset({"logs", "shutdown"})
PROBE_SECTION_KEYS = {
    "run": frozenset({
        "run_id", "run_dir", "started_utc", "probe_state",
    }),
    "environment": frozenset({
        "home", "ros_master_uri", "gazebo_master_uri",
        "gazebo_model_database_uri", "search_paths", "fixed_environment",
        "state_paths", "writable_fd_audit",
    }),
    "provenance": frozenset({
        "install_contract_path", "install_contract_sha256", "packages",
        "renderer", "input", "model", "meshes", "plugins",
        "external_before",
    }),
    "processes": frozenset({
        "owned_root", "gzserver", "models", "required_nodes",
    }),
    "graph": frozenset({"snapshot_stamp", "topics"}),
    "samples": frozenset({"clock", "odom", "scan"}),
    "guard": frozenset({
        "records", "probe_disconnected", "post_probe_snapshot_stamp",
    }),
    "motion": frozenset({"forward", "rotation"}),
    "tf": frozenset({"edges", "forbidden_frames"}),
    "collision": COLLISION_KEYS,
}
LAUNCHER_RUN_KEYS = frozenset({
    "run_id", "run_dir", "started_utc", "finished_utc", "wall_seconds",
    "supervisor_status",
})
RESULT_RUN_KEYS = LAUNCHER_RUN_KEYS
RESULT_PROVENANCE_KEYS = (
    PROBE_SECTION_KEYS["provenance"] |
    frozenset({"external_after", "external_unchanged"}))
LOG_KEYS = frozenset({"launch_path", "launch_sha256", "fatal_matches"})
SHUTDOWN_KEYS = frozenset({
    "probe_status", "launcher_status", "cleanup_owner",
    "signal_sequence", "escalated", "remaining_pids", "remaining_ports",
})
LAUNCHER_KEYS = frozenset({
    "schema_version", "run", "logs", "shutdown", "external_after",
    "first_failure", "fatal_errors",
})
FIRST_FAILURE_KEYS = frozenset({"exit_class", "phase", "message"})
SEARCH_PATH_KEYS = frozenset({
    "ROS_PACKAGE_PATH", "CMAKE_PREFIX_PATH", "PYTHONPATH",
    "LD_LIBRARY_PATH", "GAZEBO_PLUGIN_PATH", "GAZEBO_MODEL_PATH",
    "GAZEBO_RESOURCE_PATH", "PKG_CONFIG_PATH", "OGRE_RESOURCE_PATH",
})
EXPECTED_FIXED_ENVIRONMENT = {
    "PATH": "/opt/ros/noetic/bin:/usr/bin:/bin",
    "LANG": "C.UTF-8",
    "LC_ALL": "C.UTF-8",
    "ROS_ETC_DIR": "/opt/ros/noetic/etc/ros",
    "ROS_ROOT": "/opt/ros/noetic/share/ros",
    "ROSLISP_PACKAGE_DIRECTORIES": "",
}
FIXED_ENVIRONMENT_KEYS = frozenset(EXPECTED_FIXED_ENVIRONMENT)
STATE_PATH_KEYS = frozenset({
    "TMPDIR", "ROS_HOME", "ROS_LOG_DIR", "GAZEBO_LOG_PATH",
    "IGN_FUEL_CACHE_PATH", "XDG_CACHE_HOME", "XDG_CONFIG_HOME",
    "XDG_DATA_HOME",
})
WRITABLE_AUDIT_KEYS = frozenset({
    "checked_pids", "writable_files", "allowed_kernel_endpoints",
})
FILE_RECORD_KEYS = frozenset({"path", "sha256"})
MESH_RECORD_KEYS = frozenset({"uri", "path", "sha256"})
INSTALL_CONTRACT_KEYS = frozenset({
    "schema_version", "status", "environment", "packages", "renderer",
    "meshes", "plugins",
})
INSTALL_RENDERER_KEYS = frozenset({
    "path", "sha256", "input", "input_sha256", "output",
    "output_sha256",
})
INSTALL_PLUGIN_KEYS = frozenset({"path", "ldd_status"})
INSTALL_ENVIRONMENT_KEYS = (
    SEARCH_PATH_KEYS | FIXED_ENVIRONMENT_KEYS | STATE_PATH_KEYS |
    frozenset({"GAZEBO_MODEL_DATABASE_URI"}))
PACKAGE_KEYS = frozenset({"bunker_description", "bunker_sim_runtime"})
EXTERNAL_KEYS = frozenset({"p450_sim", "px4"})
EXTERNAL_RECORD_KEYS = frozenset({"head", "status"})
GIT_HEAD_RE = re.compile(r"^[0-9a-f]{40}$")
PROCESS_ID_KEYS = frozenset({"pid", "pgid", "sid", "start_ticks"})
GZSERVER_KEYS = PROCESS_ID_KEYS | frozenset({"executable", "plugin_maps"})
CLOCK_SUMMARY_KEYS = frozenset({
    "count", "first_stamp", "last_stamp", "median_period",
    "minimum_period", "maximum_period",
})
ODOM_SUMMARY_KEYS = frozenset({
    "count", "first_stamp", "last_stamp", "median_period", "frame_id",
    "child_frame_id", "position_min", "position_max",
    "linear_twist_abs_max", "angular_twist_abs_max",
    "quaternion_norm_min", "quaternion_norm_max",
})
SCAN_SUMMARY_KEYS = frozenset({
    "count", "first_stamp", "last_stamp", "median_period", "frame_id",
    "range_count", "intensity_count", "angle_min", "angle_max",
    "angle_increment", "range_min", "range_max", "time_increment",
    "scan_time", "positive_infinity_count", "finite_range_min",
    "finite_range_max", "intensity_min", "intensity_max",
})


class LiveContractError(ValueError):
    pass


class _UsageError(ValueError):
    pass


class _ContractArgumentParser(argparse.ArgumentParser):
    def error(self, message):
        raise _UsageError(message)


def _require_exact_keys(value, expected, label):
    if type(value) is not dict:
        raise LiveContractError("%s must be an object" % label)
    actual = set(value)
    if actual != set(expected):
        raise LiveContractError(
            "%s keys differ missing=%r extra=%r" %
            (label, sorted(set(expected) - actual),
             sorted(actual - set(expected))))
    return value


def _finite_float(value, label):
    if type(value) not in (int, float):
        raise LiveContractError("%s must be a finite number" % label)
    try:
        result = float(value)
    except (OverflowError, ValueError):
        raise LiveContractError("%s must be a finite number" % label)
    if not math.isfinite(result):
        raise LiveContractError("%s must be a finite number" % label)
    return result


def _finite_sequence(value, length, label):
    if type(value) not in (list, tuple) or len(value) != length:
        raise LiveContractError("%s must contain %d numbers" % (label, length))
    return tuple(_finite_float(item, "%s[%d]" % (label, index))
                 for index, item in enumerate(value))


def _finite_mapping(value, keys, label):
    _require_exact_keys(value, keys, label)
    return {key: _finite_float(value[key], "%s.%s" % (label, key))
            for key in sorted(keys)}


def _require_finite_json(value, label="document"):
    if value is None or type(value) in (str, bool):
        return
    if type(value) in (int, float):
        _finite_float(value, label)
        return
    if type(value) is list:
        for index, item in enumerate(value):
            _require_finite_json(item, "%s[%d]" % (label, index))
        return
    if type(value) is dict:
        for key, item in value.items():
            if type(key) is not str:
                raise LiveContractError("%s has a non-string key" % label)
            _require_finite_json(item, "%s.%s" % (label, key))
        return
    raise LiveContractError("%s has a non-JSON value" % label)


def median_period(stamps, minimum_count):
    if type(minimum_count) is not int or minimum_count < 2:
        raise LiveContractError("minimum_count must be at least two")
    if type(stamps) not in (list, tuple) or len(stamps) < minimum_count:
        raise LiveContractError("not enough timestamp samples")
    values = tuple(_finite_float(value, "stamp") for value in stamps)
    periods = []
    for previous, current in zip(values, values[1:]):
        if current <= previous:
            raise LiveContractError("timestamps must be strictly increasing")
        periods.append(_finite_float(
            current - previous, "timestamp period"))
    return _finite_float(statistics.median(periods), "median period")


def validate_clock_samples(stamps):
    if type(stamps) not in (list, tuple) or len(stamps) < 3:
        raise LiveContractError("clock requires at least three samples")
    values = tuple(_finite_float(value, "clock stamp") for value in stamps)
    if any(value <= 0.0 for value in values):
        raise LiveContractError("clock stamps must be nonzero")
    period = median_period(values, 3)
    periods = tuple(current - previous
                    for previous, current in zip(values, values[1:]))
    return {
        "count": len(values), "first_stamp": values[0],
        "last_stamp": values[-1], "median_period": period,
        "minimum_period": min(periods), "maximum_period": max(periods),
    }


def validate_odometry_samples(samples):
    if type(samples) not in (list, tuple) or len(samples) < 20:
        raise LiveContractError("odometry requires at least 20 samples")
    stamps = []
    positions = []
    linear = []
    angular = []
    quaternion_norms = []
    for index, sample in enumerate(samples):
        label = "odometry[%d]" % index
        _require_exact_keys(sample, ODOMETRY_SAMPLE_KEYS, label)
        stamp = _finite_float(sample["stamp"], label + ".stamp")
        if stamp <= 0.0:
            raise LiveContractError("odometry stamp must be nonzero")
        if (sample["frame_id"] != "ground/odom" or
                sample["child_frame_id"] != "ground/base_link"):
            raise LiveContractError("odometry frames differ")
        position = _finite_mapping(
            sample["position"], VECTOR3_KEYS, label + ".position")
        orientation = _finite_mapping(
            sample["orientation"], QUATERNION_KEYS,
            label + ".orientation")
        linear_value = _finite_mapping(
            sample["linear_twist"], VECTOR3_KEYS,
            label + ".linear_twist")
        angular_value = _finite_mapping(
            sample["angular_twist"], VECTOR3_KEYS,
            label + ".angular_twist")
        norm = math.hypot(*(orientation[key]
                            for key in sorted(QUATERNION_KEYS)))
        if not math.isfinite(norm) or norm <= 1e-12:
            raise LiveContractError(
                "odometry quaternion has invalid norm")
        stamps.append(stamp)
        positions.append(position)
        linear.append(linear_value)
        angular.append(angular_value)
        quaternion_norms.append(norm)
    period = median_period(stamps, 20)
    if not 0.016 <= period <= 0.030:
        raise LiveContractError("odometry median period is outside contract")
    return {
        "count": len(samples), "first_stamp": stamps[0],
        "last_stamp": stamps[-1], "median_period": period,
        "frame_id": "ground/odom", "child_frame_id": "ground/base_link",
        "position_min": {axis: min(item[axis] for item in positions)
                         for axis in sorted(VECTOR3_KEYS)},
        "position_max": {axis: max(item[axis] for item in positions)
                         for axis in sorted(VECTOR3_KEYS)},
        "linear_twist_abs_max": max(
            abs(item[axis]) for item in linear for axis in VECTOR3_KEYS),
        "angular_twist_abs_max": max(
            abs(item[axis]) for item in angular for axis in VECTOR3_KEYS),
        "quaternion_norm_min": min(quaternion_norms),
        "quaternion_norm_max": max(quaternion_norms),
    }


def validate_scan_samples(samples):
    if type(samples) not in (list, tuple) or len(samples) < 8:
        raise LiveContractError("scan requires at least eight samples")
    stamps = []
    finite_ranges = []
    infinity_count = 0
    intensity_min = None
    intensity_max = None
    for index, sample in enumerate(samples):
        label = "scan[%d]" % index
        _require_exact_keys(sample, SCAN_SAMPLE_KEYS, label)
        stamp = _finite_float(sample["stamp"], label + ".stamp")
        if stamp <= 0.0:
            raise LiveContractError("scan stamp must be nonzero")
        if sample["frame_id"] != SCAN_CONTRACT["frame_id"]:
            raise LiveContractError("scan frame differs")
        if (type(sample["ranges"]) not in (list, tuple) or
                len(sample["ranges"]) != SCAN_CONTRACT["range_count"]):
            raise LiveContractError("scan range count differs")
        if (type(sample["intensities"]) not in (list, tuple) or
                len(sample["intensities"]) !=
                SCAN_CONTRACT["intensity_count"]):
            raise LiveContractError("scan intensity count differs")
        for key in ("angle_min", "angle_max", "angle_increment",
                    "range_min", "range_max"):
            observed = _finite_float(sample[key], label + "." + key)
            if abs(observed - SCAN_CONTRACT[key]) > 1e-6:
                raise LiveContractError("scan %s differs" % key)
        for key in ("time_increment", "scan_time"):
            observed = _finite_float(sample[key], label + "." + key)
            if abs(observed - SCAN_CONTRACT[key]) > 1e-9:
                raise LiveContractError("scan %s differs" % key)
        for range_index, value in enumerate(sample["ranges"]):
            if type(value) not in (int, float) or type(value) is bool:
                raise LiveContractError("scan range is not numeric")
            try:
                number = float(value)
            except (OverflowError, ValueError):
                raise LiveContractError("scan range is invalid")
            if math.isnan(number) or number == float("-inf"):
                raise LiveContractError("scan range is invalid")
            if number == float("inf"):
                infinity_count += 1
                continue
            if (not math.isfinite(number) or
                    number < SCAN_CONTRACT["range_min"] - 1e-6 or
                    number > SCAN_CONTRACT["range_max"] + 1e-6):
                raise LiveContractError(
                    "scan range[%d] is outside contract" % range_index)
            finite_ranges.append(number)
        for value in sample["intensities"]:
            number = _finite_float(value, "scan intensity")
            intensity_min = number if intensity_min is None else min(
                intensity_min, number)
            intensity_max = number if intensity_max is None else max(
                intensity_max, number)
        stamps.append(stamp)
    if not finite_ranges:
        raise LiveContractError("scan contains no finite return")
    period = median_period(stamps, 8)
    if not 0.050 <= period <= 0.090:
        raise LiveContractError("scan median period is outside contract")
    return {
        "count": len(samples), "first_stamp": stamps[0],
        "last_stamp": stamps[-1], "median_period": period,
        "frame_id": SCAN_CONTRACT["frame_id"],
        "range_count": SCAN_CONTRACT["range_count"],
        "intensity_count": SCAN_CONTRACT["intensity_count"],
        "angle_min": SCAN_CONTRACT["angle_min"],
        "angle_max": SCAN_CONTRACT["angle_max"],
        "angle_increment": SCAN_CONTRACT["angle_increment"],
        "range_min": SCAN_CONTRACT["range_min"],
        "range_max": SCAN_CONTRACT["range_max"],
        "time_increment": SCAN_CONTRACT["time_increment"],
        "scan_time": SCAN_CONTRACT["scan_time"],
        "positive_infinity_count": infinity_count,
        "finite_range_min": min(finite_ranges),
        "finite_range_max": max(finite_ranges),
        "intensity_min": intensity_min, "intensity_max": intensity_max,
    }


def validate_guard_admission(records):
    if type(records) not in (list, tuple) or len(records) != 3:
        raise LiveContractError("guard requires exactly three records")
    expected_cases = ("over_limit", "nonplanar", "nonfinite")
    normalized = []
    maximum_latency = 0.0
    for index, (record, expected_case) in enumerate(
            zip(records, expected_cases)):
        label = "guard[%d]" % index
        _require_exact_keys(record, GUARD_RECORD_KEYS, label)
        if record["case"] != expected_case:
            raise LiveContractError("guard case order differs")
        tokens = record["input_tokens"]
        if (type(tokens) is not list or
                any(type(token) is not str for token in tokens) or
                tuple(tokens) != GUARD_CASE_TOKENS[expected_case]):
            raise LiveContractError("guard input tokens differ")
        output = _finite_sequence(record["output"], 6, label + ".output")
        expected_output = GUARD_EXPECTED_OUTPUTS[expected_case]
        if any(abs(observed - expected) > 1e-9
               for observed, expected in zip(output, expected_output)):
            raise LiveContractError("guard output differs")
        input_stamp = _finite_float(
            record["input_stamp"], label + ".input_stamp")
        output_stamp = _finite_float(
            record["output_stamp"], label + ".output_stamp")
        latency = _finite_float(record["latency"], label + ".latency")
        if (input_stamp <= 0.0 or output_stamp < input_stamp or
                not 0.0 <= latency <= 0.10 or
                abs(latency - (output_stamp - input_stamp)) > 1e-9):
            raise LiveContractError("guard latency differs")
        maximum_latency = max(maximum_latency, latency)
        normalized.append({
            "case": expected_case, "input_tokens": list(tokens),
            "output": list(output), "input_stamp": input_stamp,
            "output_stamp": output_stamp, "latency": latency,
        })
    return {
        "count": 3, "cases": list(expected_cases),
        "maximum_latency": maximum_latency, "records": normalized,
    }


def unwrap_yaw(previous, current):
    previous_value = _finite_float(previous, "previous yaw")
    current_value = _finite_float(current, "current yaw")
    raw_delta = _finite_float(
        current_value - previous_value, "yaw difference")
    delta = (raw_delta + math.pi) % (2.0 * math.pi)
    delta -= math.pi
    delta = _finite_float(delta, "unwrapped yaw")
    if delta == -math.pi and raw_delta > 0.0:
        delta = math.pi
    return delta


def project_local_delta(baseline_pose, final_pose):
    _require_exact_keys(baseline_pose, {"x", "y", "yaw"}, "baseline pose")
    _require_exact_keys(final_pose, {"x", "y", "yaw"}, "final pose")
    baseline = {key: _finite_float(baseline_pose[key], "baseline." + key)
                for key in ("x", "y", "yaw")}
    final = {key: _finite_float(final_pose[key], "final." + key)
             for key in ("x", "y", "yaw")}
    dx = _finite_float(final["x"] - baseline["x"], "projected x delta")
    dy = _finite_float(final["y"] - baseline["y"], "projected y delta")
    cosine = math.cos(baseline["yaw"])
    sine = math.sin(baseline["yaw"])
    result = {
        "forward": cosine * dx + sine * dy,
        "lateral": -sine * dx + cosine * dy,
        "yaw": unwrap_yaw(baseline["yaw"], final["yaw"]),
    }
    return {key: _finite_float(value, "projected " + key)
            for key, value in result.items()}


def _validated_pose_record(value, label):
    _require_exact_keys(value, POSE_KEYS, label)
    model = _finite_mapping(value["model"], MODEL_POSE_KEYS, label + ".model")
    odom = _finite_mapping(value["odom"], ODOM_POSE_KEYS, label + ".odom")
    return {"model": model, "odom": odom}


def _close(observed, expected, tolerance=1e-9):
    return abs(observed - expected) <= tolerance


def validate_motion_phase(kind, evidence):
    if kind not in ("forward", "rotation") or type(kind) is not str:
        raise LiveContractError("unknown motion phase")
    _require_exact_keys(evidence, MOTION_PHASE_KEYS, "motion phase")
    if evidence["kind"] != kind:
        raise LiveContractError("motion kind differs")
    command = evidence["command"]
    _require_exact_keys(command, COMMAND_KEYS, "motion command")
    command_numbers = {
        key: _finite_float(command[key], "command." + key)
        for key in COMMAND_KEYS if key != "published_count"
    }
    count = command["published_count"]
    if type(count) is not int or count < (40 if kind == "forward" else 30):
        raise LiveContractError("motion publish count is too low")
    if not _close(command_numbers["rate_hz"], 20.0):
        raise LiveContractError("motion rate differs")
    start = command_numbers["start_stamp"]
    last = command_numbers["last_publish_stamp"]
    end = command_numbers["end_stamp"]
    target = command_numbers["target_duration"]
    actual = command_numbers["actual_duration"]
    if min(start, last, end) <= 0.0 or not start <= last <= end:
        raise LiveContractError("motion command timestamps differ")
    if not _close(actual, last - start):
        raise LiveContractError("motion actual duration differs")
    if end - start < target - 1e-9:
        raise LiveContractError("motion window ended before target")
    if not target - 0.075 <= actual <= target + 0.025:
        raise LiveContractError("motion publish duration differs")
    if not 0.0 <= end - last <= 0.075:
        raise LiveContractError("motion command tail differs")
    if (command_numbers["minimum_period"] < 0.025 or
            not 0.045 <= command_numbers["median_period"] <= 0.055 or
            command_numbers["maximum_period"] > 0.075 or
            not (command_numbers["minimum_period"] <=
                 command_numbers["median_period"] <=
                 command_numbers["maximum_period"])):
        raise LiveContractError("motion command timing differs")
    if kind == "forward":
        if (not _close(command_numbers["linear_x"], 0.25) or
                not _close(command_numbers["angular_z"], 0.0) or
                not _close(target, 2.0)):
            raise LiveContractError("forward command differs")
    else:
        if (not _close(command_numbers["linear_x"], 0.0) or
                not _close(command_numbers["angular_z"], 0.5) or
                not _close(target, 1.5)):
            raise LiveContractError("rotation command differs")

    baseline = _validated_pose_record(evidence["baseline"], "baseline")
    final = _validated_pose_record(evidence["final"], "final")
    watchdog = evidence["watchdog"]
    _require_exact_keys(watchdog, WATCHDOG_KEYS, "watchdog")
    watchdog_numbers = {
        key: _finite_float(watchdog[key], "watchdog." + key)
        for key in WATCHDOG_KEYS
    }
    if not _close(watchdog_numbers["last_command_stamp"], last):
        raise LiveContractError("watchdog last command differs")
    stop_stamp = watchdog_numbers["stop_observed_stamp"]
    if stop_stamp < last:
        raise LiveContractError("watchdog stopped before last command")
    if not _close(watchdog_numbers["time_to_stop"], stop_stamp - last):
        raise LiveContractError("watchdog time-to-stop differs")
    if watchdog_numbers["clock_coast"] < STOP_LIMITS["minimum_clock_coast"]:
        raise LiveContractError("watchdog clock coast is too short")
    if watchdog_numbers["time_to_stop"] < 0.0:
        raise LiveContractError("watchdog stop time is negative")
    for key in (
            "model_linear_speed", "model_angular_speed",
            "odom_linear_speed", "odom_angular_speed"):
        if not 0.0 <= watchdog_numbers[key] <= STOP_LIMITS[key + "_max"]:
            raise LiveContractError("watchdog %s differs" % key)

    stationary = evidence["stationary"]
    _require_exact_keys(stationary, STATIONARY_KEYS, "stationary")
    stationary_numbers = {
        key: _finite_float(stationary[key], "stationary." + key)
        for key in STATIONARY_KEYS
    }
    if stationary_numbers["start_stamp"] < stop_stamp:
        raise LiveContractError("stationary window begins before stop")
    if not _close(
            stationary_numbers["duration"],
            stationary_numbers["end_stamp"] -
            stationary_numbers["start_stamp"]):
        raise LiveContractError("stationary duration differs")
    if stationary_numbers["duration"] < STOP_LIMITS["stationary_window"]:
        raise LiveContractError("stationary window is too short")
    if not 0.0 <= stationary_numbers["position_drift"] <= \
            STOP_LIMITS["stationary_position_drift_max"]:
        raise LiveContractError("stationary position drift differs")
    if not 0.0 <= abs(stationary_numbers["yaw_drift"]) <= \
            STOP_LIMITS["stationary_yaw_drift_max"]:
        raise LiveContractError("stationary yaw drift differs")

    metric_keys = (FORWARD_METRIC_KEYS if kind == "forward"
                   else ROTATION_METRIC_KEYS)
    metrics = _finite_mapping(evidence["metrics"], metric_keys, "metrics")
    derived_height_change = final["model"]["z"] - baseline["model"]["z"]
    if (not _close(metrics["height_change"], derived_height_change, 1e-6) or
            abs(metrics["height_change"]) > 0.03):
        raise LiveContractError("model height changed")
    if (not _close(metrics["roll"], final["model"]["roll"], 1e-6) or
            not _close(metrics["pitch"], final["model"]["pitch"], 1e-6) or
            abs(baseline["model"]["roll"]) >= 0.05 or
            abs(baseline["model"]["pitch"]) >= 0.05 or
            abs(metrics["roll"]) >= 0.05 or abs(metrics["pitch"]) >= 0.05):
        raise LiveContractError("model attitude differs")
    if metrics["model_odom_xy"] < 0.0 or metrics["model_odom_yaw"] < 0.0:
        raise LiveContractError("model/odom error is negative")
    if kind == "forward":
        if not FORWARD_LIMITS["forward"][0] <= metrics["model_forward"] <= \
                FORWARD_LIMITS["forward"][1]:
            raise LiveContractError("forward displacement differs")
        if abs(metrics["model_lateral"]) > FORWARD_LIMITS["abs_lateral_max"]:
            raise LiveContractError("forward lateral drift differs")
        projected_model = project_local_delta(
            {key: baseline["model"][key] for key in ("x", "y", "yaw")},
            {key: final["model"][key] for key in ("x", "y", "yaw")})
        projected_odom = project_local_delta(
            baseline["odom"], final["odom"])
        for key, observed in (
                ("model_forward", projected_model["forward"]),
                ("model_lateral", projected_model["lateral"]),
                ("odom_forward", projected_odom["forward"]),
                ("odom_lateral", projected_odom["lateral"])):
            if not _close(metrics[key], observed, 1e-6):
                raise LiveContractError("forward metric %s differs" % key)
        limits = FORWARD_LIMITS
    else:
        projected_model = project_local_delta(
            {key: baseline["model"][key] for key in ("x", "y", "yaw")},
            {key: final["model"][key] for key in ("x", "y", "yaw")})
        projected_odom = project_local_delta(baseline["odom"], final["odom"])
        if not ROTATION_LIMITS["yaw"][0] <= metrics["model_yaw"] <= \
                ROTATION_LIMITS["yaw"][1]:
            raise LiveContractError("rotation yaw differs")
        if metrics["model_planar_drift"] > \
                ROTATION_LIMITS["planar_drift_max"]:
            raise LiveContractError("rotation planar drift differs")
        if (not _close(metrics["model_yaw"], projected_model["yaw"], 1e-6) or
                not _close(metrics["odom_yaw"], projected_odom["yaw"], 1e-6) or
                not _close(metrics["model_planar_drift"], math.hypot(
                    final["model"]["x"] - baseline["model"]["x"],
                    final["model"]["y"] - baseline["model"]["y"]), 1e-6)):
            raise LiveContractError("rotation derived metrics differ")
        limits = ROTATION_LIMITS
    model_odom_xy = math.hypot(
        final["model"]["x"] - final["odom"]["x"],
        final["model"]["y"] - final["odom"]["y"])
    model_odom_yaw = abs(unwrap_yaw(
        final["model"]["yaw"], final["odom"]["yaw"]))
    if (not _close(metrics["model_odom_xy"], model_odom_xy, 1e-6) or
            not _close(metrics["model_odom_yaw"], model_odom_yaw, 1e-6)):
        raise LiveContractError("model/odom derived agreement differs")
    if metrics["model_odom_xy"] > limits["model_odom_xy_max"]:
        raise LiveContractError("model/odom xy agreement differs")
    if metrics["model_odom_yaw"] > limits["model_odom_yaw_max"]:
        raise LiveContractError("model/odom yaw agreement differs")
    normalized = copy.deepcopy(evidence)
    _require_finite_json(normalized, "motion phase")
    return normalized


def validate_graph(evidence):
    _require_exact_keys(
        evidence, {"gzserver_count", "nodes", "models", "topics"},
        "graph")
    if type(evidence["gzserver_count"]) is not int or \
            evidence["gzserver_count"] != 1:
        raise LiveContractError("graph requires one gzserver")
    if evidence["nodes"] != ["/gazebo", "/ground/velocity_guard"]:
        raise LiveContractError("graph node set differs")
    if evidence["models"] != ["bunker"]:
        raise LiveContractError("graph model set differs")
    topics = evidence["topics"]
    _require_exact_keys(topics, frozenset(EXPECTED_GRAPH), "graph topics")
    normalized_topics = {}
    for name in sorted(EXPECTED_GRAPH):
        record = topics[name]
        _require_exact_keys(
            record, {"type", "publishers", "subscribers"},
            "graph topic " + name)
        expected = EXPECTED_GRAPH[name]
        if record["type"] != expected["type"]:
            raise LiveContractError("graph topic type differs for " + name)
        normalized = {"type": record["type"]}
        for field in ("publishers", "subscribers"):
            owners = record[field]
            if (type(owners) is not list or
                    any(type(owner) is not str for owner in owners) or
                    len(owners) != len(set(owners)) or
                    frozenset(owners) != expected[field]):
                raise LiveContractError(
                    "graph %s differ for %s" % (field, name))
            normalized[field] = sorted(owners)
        normalized_topics[name] = normalized
    return {
        "gzserver_count": 1,
        "nodes": list(evidence["nodes"]),
        "models": list(evidence["models"]),
        "topic_count": len(normalized_topics),
        "topics": normalized_topics,
    }


def _canonical_regular_file(value, label):
    if not isinstance(value, (str, os.PathLike)):
        raise LiveContractError("%s must be a path" % label)
    candidate = Path(value)
    if not candidate.is_absolute():
        raise LiveContractError("%s must be absolute" % label)
    try:
        resolved = candidate.resolve(strict=True)
    except (OSError, RuntimeError) as error:
        raise LiveContractError("%s is unavailable: %s" % (label, error))
    if candidate != resolved or candidate.is_symlink() or not resolved.is_file():
        raise LiveContractError("%s must be a canonical regular file" % label)
    return resolved


def _sha256_file(path):
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            while True:
                block = stream.read(1024 * 1024)
                if not block:
                    break
                digest.update(block)
    except OSError as error:
        raise LiveContractError("cannot hash %s: %s" % (path, error))
    return digest.hexdigest()


def _rpy_quaternion(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    values = (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-12:
        raise LiveContractError("URDF rotation has zero norm")
    return tuple(value / norm for value in values)


def _parse_triplet(value, label):
    text_value = "0 0 0" if value is None else value
    if type(text_value) is not str:
        raise LiveContractError("%s must be text" % label)
    parts = text_value.split()
    if len(parts) != 3:
        raise LiveContractError("%s must contain three numbers" % label)
    try:
        values = tuple(float(part) for part in parts)
    except ValueError:
        raise LiveContractError("%s contains a non-number" % label)
    if not all(math.isfinite(item) for item in values):
        raise LiveContractError("%s contains a non-finite number" % label)
    return values


def load_expected_fixed_transforms(rendered_urdf_path, expected_sha256):
    path = _canonical_regular_file(rendered_urdf_path, "rendered URDF")
    if type(expected_sha256) is not str or not SHA256_RE.fullmatch(
            expected_sha256):
        raise LiveContractError("expected URDF hash is malformed")
    if expected_sha256 != RUNTIME_URDF_SHA256:
        raise LiveContractError("expected URDF hash is not the frozen runtime")
    run_dir = _validate_run_directory(path.parent)
    if (path != run_dir / "rendered-bunker.urdf" or
            stat.S_IMODE(path.stat().st_mode) != 0o600):
        raise LiveContractError("rendered URDF boundary differs")
    if _sha256_file(path) != expected_sha256:
        raise LiveContractError("rendered URDF hash differs")
    try:
        root = ET.parse(str(path)).getroot()
    except (ET.ParseError, OSError) as error:
        raise LiveContractError("rendered URDF is invalid: %s" % error)
    joints = list(root.findall("joint"))
    if len(joints) != 17 or any(joint.get("type") != "fixed"
                                for joint in joints):
        raise LiveContractError("rendered URDF fixed-joint count differs")
    expected = {}
    for joint in joints:
        parent_element = joint.find("parent")
        child_element = joint.find("child")
        if parent_element is None or child_element is None:
            raise LiveContractError("rendered URDF joint is incomplete")
        parent_local = parent_element.get("link")
        child_local = child_element.get("link")
        if parent_local != "base_link" or type(child_local) is not str:
            raise LiveContractError("rendered URDF joint topology differs")
        key = ("ground/" + parent_local, "ground/" + child_local,
               "tf_static")
        if key in expected:
            raise LiveContractError("rendered URDF has duplicate TF child")
        origin = joint.find("origin")
        translation = _parse_triplet(
            None if origin is None else origin.get("xyz"),
            "URDF joint xyz")
        rpy = _parse_triplet(
            None if origin is None else origin.get("rpy"),
            "URDF joint rpy")
        expected[key] = {
            "translation": translation,
            "rotation": _rpy_quaternion(*rpy),
        }
    if frozenset(key[1] for key in expected) != \
            frozenset(EXPECTED_FIXED_CHILDREN):
        raise LiveContractError("rendered URDF fixed children differ")
    return expected


def _validate_frame_name(value, label):
    if type(value) is not str or not value or value.startswith("/"):
        raise LiveContractError("%s is invalid" % label)
    if value != "world" and not value.startswith("ground/"):
        raise LiveContractError("%s is unprefixed" % label)
    if "ground/ground/" in value:
        raise LiveContractError("%s is double-prefixed" % label)
    return value


def validate_tf_authorities(edges, current_clock,
                            expected_fixed_transforms):
    clock = _finite_float(current_clock, "current clock")
    if clock <= 0.0:
        raise LiveContractError("current clock must be nonzero")
    if type(expected_fixed_transforms) is not dict or \
            len(expected_fixed_transforms) != 17:
        raise LiveContractError("expected fixed transform map differs")
    if type(edges) is not list or len(edges) != 19:
        raise LiveContractError("TF requires exactly 19 edges")
    observed = {}
    for index, edge in enumerate(edges):
        label = "tf edge[%d]" % index
        _require_exact_keys(edge, TF_EDGE_KEYS, label)
        parent = _validate_frame_name(edge["parent"], label + ".parent")
        child = _validate_frame_name(edge["child"], label + ".child")
        channel = edge["channel"]
        if channel not in ("tf", "tf_static"):
            raise LiveContractError("TF channel differs")
        key = (parent, child, channel)
        if key in observed:
            raise LiveContractError("TF edge is duplicated")
        if key not in EXPECTED_TF_AUTHORITIES:
            raise LiveContractError("unexpected TF edge")
        if edge["authority"] != EXPECTED_TF_AUTHORITIES[key]:
            raise LiveContractError("TF authority differs")
        stamp = _finite_float(edge["stamp"], label + ".stamp")
        translation = _finite_sequence(
            edge["translation"], 3, label + ".translation")
        rotation = _finite_sequence(edge["rotation"], 4, label + ".rotation")
        norm = math.sqrt(sum(item * item for item in rotation))
        if norm <= 1e-12 or abs(norm - 1.0) > 1e-6:
            raise LiveContractError("TF quaternion is not normalized")
        observed[key] = {
            "translation": translation, "rotation": rotation,
            "stamp": stamp,
        }
    if frozenset(observed) != frozenset(EXPECTED_TF_AUTHORITIES):
        raise LiveContractError("TF edge set differs")
    world_key = ("world", "ground/odom", "tf_static")
    world_edge = observed[world_key]
    if (any(abs(value) > 1e-9 for value in world_edge["translation"]) or
            any(abs(value - expected) > 1e-9
                for value, expected in zip(
                    world_edge["rotation"], (0.0, 0.0, 0.0, 1.0))) or
            abs(world_edge["stamp"]) > 1e-9):
        raise LiveContractError("world TF must be identity")
    dynamic = observed[("ground/odom", "ground/base_link", "tf")]
    if not 0.0 <= clock - dynamic["stamp"] <= 0.10:
        raise LiveContractError("dynamic TF is stale or future-dated")
    for key, expected in expected_fixed_transforms.items():
        if key not in observed or key not in EXPECTED_TF_AUTHORITIES:
            raise LiveContractError("fixed TF edge differs")
        actual = observed[key]
        if abs(actual["stamp"]) > 1e-9:
            raise LiveContractError("fixed TF stamp differs")
        if (any(abs(a - b) > 1e-6 for a, b in zip(
                actual["translation"], expected["translation"])) or
                any(abs(a - b) > 1e-6 for a, b in zip(
                    actual["rotation"], expected["rotation"]))):
            raise LiveContractError("fixed TF numeric value differs")
    parents = {child: parent for parent, child, _channel in observed}
    if len(parents) != 19:
        raise LiveContractError("TF child has multiple parents")
    for child in parents:
        cursor = child
        visited = set()
        while cursor != "world":
            if cursor in visited or cursor not in parents:
                raise LiveContractError("TF graph is disconnected or cyclic")
            visited.add(cursor)
            cursor = parents[cursor]
    return {
        "edge_count": 19,
        "authorities": sorted(set(EXPECTED_TF_AUTHORITIES.values())),
        "root": "world",
        "dynamic_age": clock - dynamic["stamp"],
    }


def validate_plugin_maps(records):
    if type(records) is not list or len(records) != len(PLUGIN_PATHS):
        raise LiveContractError("plugin map count differs")
    observed = {}
    for index, record in enumerate(records):
        _require_exact_keys(record, PLUGIN_RECORD_KEYS,
                            "plugin map[%d]" % index)
        basename = record["basename"]
        if type(basename) is not str or not basename:
            raise LiveContractError("plugin basename is invalid")
        path = _canonical_regular_file(
            record["path"], "plugin map[%d].path" % index)
        if basename != path.name or basename in observed:
            raise LiveContractError("plugin basename is duplicated or differs")
        observed[basename] = str(path)
    expected = {Path(path).name: path for path in PLUGIN_PATHS}
    if observed != expected:
        raise LiveContractError("plugin map paths differ")
    return {
        "count": len(observed),
        "records": [
            {"basename": basename, "path": observed[basename]}
            for basename in sorted(observed)
        ],
    }


def validate_collision(evidence):
    _require_exact_keys(evidence, COLLISION_KEYS, "collision")
    if evidence["model"] != "bunker" or evidence["link"] != "base_link":
        raise LiveContractError("collision model or link differs")
    if type(evidence["mesh_collision_count"]) is not int or \
            evidence["mesh_collision_count"] != 0:
        raise LiveContractError("mesh collision count differs")
    collisions = evidence["collisions"]
    if type(collisions) is not list or len(collisions) != 1:
        raise LiveContractError("collision record count differs")
    record = collisions[0]
    _require_exact_keys(record, COLLISION_RECORD_KEYS, "collision record")
    if (record["name"] != "base_link_collision" or
            record["link"] != "base_link" or record["geometry"] != "box"):
        raise LiveContractError("collision record identity differs")
    origin = _finite_sequence(record["origin"], 3, "collision origin")
    size = _finite_sequence(record["size"], 3, "collision size")
    if (any(abs(actual - expected) > 1e-6
            for actual, expected in zip(origin, COLLISION_ORIGIN)) or
            any(abs(actual - expected) > 1e-6
                for actual, expected in zip(size, COLLISION_SIZE))):
        raise LiveContractError("collision dimensions differ")
    baseline = _finite_float(evidence["baseline_height"],
                             "collision baseline height")
    final = _finite_float(evidence["final_height"],
                          "collision final height")
    roll = _finite_float(evidence["roll"], "collision roll")
    pitch = _finite_float(evidence["pitch"], "collision pitch")
    if abs(final - baseline) > 0.03:
        raise LiveContractError("collision model height differs")
    if abs(roll) >= 0.05 or abs(pitch) >= 0.05:
        raise LiveContractError("collision model attitude differs")
    raw_path = _canonical_regular_file(
        evidence["raw_info_path"], "collision raw info")
    digest = evidence["raw_info_sha256"]
    if type(digest) is not str or not SHA256_RE.fullmatch(digest):
        raise LiveContractError("collision raw-info hash is malformed")
    if _sha256_file(raw_path) != digest:
        raise LiveContractError("collision raw-info hash differs")
    return {
        "claim": "proxy-only",
        "model": "bunker", "link": "base_link",
        "collisions": [{
            "name": "base_link_collision", "link": "base_link",
            "geometry": "box", "origin": list(origin), "size": list(size),
        }],
        "mesh_collision_count": 0,
        "baseline_height": baseline, "final_height": final,
        "roll": roll, "pitch": pitch,
        "raw_info_path": str(raw_path), "raw_info_sha256": digest,
    }


def _canonical_directory(value, label):
    if not isinstance(value, (str, os.PathLike)):
        raise LiveContractError("%s must be a path" % label)
    candidate = Path(value)
    if not candidate.is_absolute():
        raise LiveContractError("%s must be absolute" % label)
    try:
        resolved = candidate.resolve(strict=True)
    except (OSError, RuntimeError) as error:
        raise LiveContractError("%s is unavailable: %s" % (label, error))
    if candidate != resolved or candidate.is_symlink() or not resolved.is_dir():
        raise LiveContractError("%s must be a canonical directory" % label)
    return resolved


def _is_descendant(path, root):
    try:
        path.relative_to(root)
        return path != root
    except ValueError:
        return False


def _expected_search_paths():
    install = str(INSTALL_ROOT)
    return {
        "ROS_PACKAGE_PATH": install + "/share:/opt/ros/noetic/share",
        "CMAKE_PREFIX_PATH": install + ":/opt/ros/noetic",
        "PYTHONPATH": (
            install + "/lib/python3/dist-packages:"
            "/opt/ros/noetic/lib/python3/dist-packages"),
        "LD_LIBRARY_PATH": (
            install + "/lib:/opt/ros/noetic/lib:"
            "/opt/ros/noetic/lib/x86_64-linux-gnu:"
            "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins"),
        "GAZEBO_PLUGIN_PATH": (
            "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:"
            "/opt/ros/noetic/lib"),
        "GAZEBO_MODEL_PATH": "/usr/share/gazebo-11/models",
        "GAZEBO_RESOURCE_PATH": "/usr/share/gazebo-11",
        "PKG_CONFIG_PATH": (
            install + "/lib/pkgconfig:/opt/ros/noetic/lib/pkgconfig:"
            "/opt/ros/noetic/lib/x86_64-linux-gnu/pkgconfig"),
        "OGRE_RESOURCE_PATH": "/usr/lib/x86_64-linux-gnu/OGRE-1.9.0",
    }


def _validate_pid_list(value, label):
    if (type(value) is not list or any(
            type(pid) is not int or pid <= 0 for pid in value) or
            len(value) != len(set(value))):
        raise LiveContractError("%s differs" % label)
    return sorted(value)


def validate_environment(evidence):
    _require_exact_keys(evidence, PROBE_SECTION_KEYS["environment"],
                        "environment")
    home = _canonical_directory(evidence["home"], "environment home")
    if home != Path.home().resolve():
        raise LiveContractError("environment home differs")
    master_ports = []
    for key in ("ros_master_uri", "gazebo_master_uri"):
        value = evidence[key]
        if type(value) is not str:
            raise LiveContractError("%s is invalid" % key)
        match = URI_RE.fullmatch(value)
        if match is None or int(match.group(1)) > 65535:
            raise LiveContractError("%s is invalid" % key)
        master_ports.append(int(match.group(1)))
    if len(set(master_ports)) != 2:
        raise LiveContractError("master ports must be distinct")
    if evidence["gazebo_model_database_uri"] != "":
        raise LiveContractError("Gazebo model database must be disabled")
    search_paths = evidence["search_paths"]
    _require_exact_keys(search_paths, SEARCH_PATH_KEYS,
                        "environment search paths")
    if search_paths != _expected_search_paths():
        raise LiveContractError("environment search paths differ")
    fixed = evidence["fixed_environment"]
    _require_exact_keys(fixed, FIXED_ENVIRONMENT_KEYS,
                        "fixed environment")
    if fixed != EXPECTED_FIXED_ENVIRONMENT:
        raise LiveContractError("fixed environment differs")
    state_paths = evidence["state_paths"]
    _require_exact_keys(state_paths, STATE_PATH_KEYS,
                        "environment state paths")
    resolved_state = {
        key: _canonical_directory(value, "state path " + key)
        for key, value in state_paths.items()
    }
    run_dirs = {path.parent for path in resolved_state.values()}
    if len(run_dirs) != 1:
        raise LiveContractError("state paths do not share a run directory")
    run_dir = next(iter(run_dirs))
    run_root = _canonical_directory(RUN_ROOT, "run root")
    if not _is_descendant(run_dir, run_root):
        raise LiveContractError("run directory is outside run root")
    if stat.S_IMODE(run_dir.stat().st_mode) != 0o700:
        raise LiveContractError("run directory mode differs")
    for path in resolved_state.values():
        if path.parent != run_dir or stat.S_IMODE(path.stat().st_mode) != 0o700:
            raise LiveContractError("state path boundary differs")
    audit = evidence["writable_fd_audit"]
    _require_exact_keys(audit, WRITABLE_AUDIT_KEYS, "writable fd audit")
    checked_pids = _validate_pid_list(audit["checked_pids"],
                                      "writable fd audit PIDs")
    writable_files = audit["writable_files"]
    endpoints = audit["allowed_kernel_endpoints"]
    if (type(writable_files) is not list or writable_files or
            type(endpoints) is not list or
            any(type(item) is not str or not item for item in endpoints) or
            len(endpoints) != len(set(endpoints))):
        raise LiveContractError("writable fd audit differs")
    return {
        "run_dir": str(run_dir), "home": str(home),
        "master_ports": master_ports,
        "search_paths": dict(search_paths),
        "fixed_environment": dict(fixed),
        "state_paths": {key: str(resolved_state[key])
                        for key in sorted(resolved_state)},
        "writable_fd_audit": {
            "checked_pids": checked_pids, "writable_files": [],
            "allowed_kernel_endpoints": list(endpoints),
        },
    }


def _validate_file_record(record, label):
    _require_exact_keys(record, FILE_RECORD_KEYS, label)
    path = _canonical_regular_file(record["path"], label + ".path")
    digest = record["sha256"]
    if type(digest) is not str or SHA256_RE.fullmatch(digest) is None:
        raise LiveContractError("%s hash is malformed" % label)
    if _sha256_file(path) != digest:
        raise LiveContractError("%s hash differs" % label)
    return {"path": str(path), "sha256": digest}


def _validate_external_snapshot(snapshot, label, require_clean=True):
    _require_exact_keys(snapshot, EXTERNAL_KEYS, label)
    normalized = {}
    for name in sorted(EXTERNAL_KEYS):
        record = snapshot[name]
        _require_exact_keys(record, EXTERNAL_RECORD_KEYS,
                            "%s.%s" % (label, name))
        if (type(record["head"]) is not str or
                GIT_HEAD_RE.fullmatch(record["head"]) is None or
                type(record["status"]) is not str or
                (require_clean and record["status"] != "")):
            raise LiveContractError(
                "%s.%s is dirty or malformed" % (label, name))
        normalized[name] = dict(record)
    return normalized


def _validate_mesh_records(records, package):
    if type(records) is not list or len(records) != 17:
        raise LiveContractError("provenance mesh count differs")
    normalized = []
    seen_uris = set()
    seen_paths = set()
    prefix = "package://bunker_description/"
    for index, record in enumerate(records):
        label = "provenance mesh[%d]" % index
        _require_exact_keys(record, MESH_RECORD_KEYS, label)
        uri = record["uri"]
        if type(uri) is not str or not uri.startswith(prefix):
            raise LiveContractError("provenance mesh URI differs")
        relative = uri[len(prefix):]
        if (not relative.startswith("meshes/") or relative.startswith("/") or
                ".." in Path(relative).parts):
            raise LiveContractError("provenance mesh relative path differs")
        file_record = _validate_file_record({
            "path": record["path"], "sha256": record["sha256"],
        }, label)
        expected_path = package / relative
        if Path(file_record["path"]) != expected_path:
            raise LiveContractError("provenance mesh path differs")
        if uri in seen_uris or file_record["path"] in seen_paths:
            raise LiveContractError("provenance mesh is duplicated")
        seen_uris.add(uri)
        seen_paths.add(file_record["path"])
        normalized.append({
            "uri": uri, "path": file_record["path"],
            "sha256": file_record["sha256"],
        })
    return sorted(normalized, key=lambda item: item["uri"])


def _validate_install_environment(environment, run_dir):
    _require_exact_keys(environment, INSTALL_ENVIRONMENT_KEYS,
                        "install environment")
    for key, expected in _expected_search_paths().items():
        if environment[key] != expected:
            raise LiveContractError("install search environment differs")
    for key, expected in EXPECTED_FIXED_ENVIRONMENT.items():
        if environment[key] != expected:
            raise LiveContractError("install fixed environment differs")
    if environment["GAZEBO_MODEL_DATABASE_URI"] != "":
        raise LiveContractError("install Gazebo model database is enabled")
    for key in STATE_PATH_KEYS:
        path = _canonical_directory(environment[key],
                                    "install state path " + key)
        if (path.parent != run_dir or
                stat.S_IMODE(path.stat().st_mode) != 0o700):
            raise LiveContractError("install state environment differs")


def _load_runtime_mesh_uris(model_path):
    try:
        root = ET.parse(str(model_path)).getroot()
    except (ET.ParseError, OSError) as error:
        raise LiveContractError("runtime model mesh closure is unreadable: %s" %
                                error)
    uris = []
    for element in root.findall(".//mesh"):
        uri = element.get("filename")
        if type(uri) is not str or not uri.startswith(
                "package://bunker_description/meshes/"):
            raise LiveContractError("runtime model mesh URI differs")
        uris.append(uri)
    if len(uris) != 17 or len(set(uris)) != 17:
        raise LiveContractError("runtime model mesh URI count differs")
    return frozenset(uris)


def validate_provenance(evidence):
    _require_exact_keys(evidence, PROBE_SECTION_KEYS["provenance"],
                        "provenance")
    contract_path = _canonical_regular_file(
        evidence["install_contract_path"], "install contract")
    contract_digest = evidence["install_contract_sha256"]
    if (type(contract_digest) is not str or
            SHA256_RE.fullmatch(contract_digest) is None or
            _sha256_file(contract_path) != contract_digest):
        raise LiveContractError("install contract hash differs")
    run_dir = _validate_run_directory(contract_path.parent)
    if (contract_path != run_dir / "install-contract.json" or
            stat.S_IMODE(contract_path.stat().st_mode) != 0o600):
        raise LiveContractError("install contract boundary differs")
    contract = _load_json_document(contract_path, "install contract")
    _require_exact_keys(contract, INSTALL_CONTRACT_KEYS, "install contract")
    if (type(contract["schema_version"]) is not int or
            contract["schema_version"] != 1 or contract["status"] != "PASS"):
        raise LiveContractError("install contract status differs")
    _validate_install_environment(contract["environment"], run_dir)

    packages = evidence["packages"]
    _require_exact_keys(packages, PACKAGE_KEYS, "provenance packages")
    contract_packages = contract["packages"]
    _require_exact_keys(contract_packages, PACKAGE_KEYS,
                        "install contract packages")
    expected_packages = {
        "bunker_description": str(
            INSTALL_ROOT / "share/bunker_description"),
        "bunker_sim_runtime": str(
            INSTALL_ROOT / "share/bunker_sim_runtime"),
    }
    normalized_packages = {}
    for name in sorted(PACKAGE_KEYS):
        path = _canonical_directory(packages[name], "package " + name)
        if str(path) != expected_packages[name] or \
                contract_packages[name] != str(path):
            raise LiveContractError("installed package provenance differs")
        normalized_packages[name] = str(path)

    renderer = _validate_file_record(evidence["renderer"], "renderer")
    source = _validate_file_record(evidence["input"], "renderer input")
    model = _validate_file_record(evidence["model"], "runtime model")
    expected_renderer_path = (
        INSTALL_ROOT / "share/bunker_sim_runtime/scripts/"
        "render_bunker_runtime.py")
    expected_source_path = (
        INSTALL_ROOT / "share/bunker_description/urdf/"
        "bunker.urdf.xacro")
    if Path(renderer["path"]) != expected_renderer_path:
        raise LiveContractError("renderer path differs")
    if (Path(source["path"]) != expected_source_path or
            source["sha256"] != FROZEN_XACRO_SHA256):
        raise LiveContractError("renderer input provenance differs")
    if (Path(model["path"]).parent != run_dir or
            Path(model["path"]).name != "rendered-bunker.urdf" or
            model["sha256"] != RUNTIME_URDF_SHA256 or
            stat.S_IMODE(Path(model["path"]).stat().st_mode) != 0o600):
        raise LiveContractError("runtime model provenance differs")
    load_expected_fixed_transforms(model["path"], model["sha256"])

    install_renderer = contract["renderer"]
    _require_exact_keys(install_renderer, INSTALL_RENDERER_KEYS,
                        "install renderer")
    if (install_renderer["path"] != renderer["path"] or
            install_renderer["sha256"] != renderer["sha256"] or
            install_renderer["input"] != source["path"] or
            install_renderer["input_sha256"] != source["sha256"] or
            install_renderer["output_sha256"] != model["sha256"]):
        raise LiveContractError("install renderer chain differs")
    output = _canonical_regular_file(
        install_renderer["output"], "install rendered output")
    if (output.parent != run_dir or
            output.name != "install-bunker-runtime.urdf" or
            stat.S_IMODE(output.stat().st_mode) != 0o600 or
            _sha256_file(output) != model["sha256"]):
        raise LiveContractError("install rendered output differs")

    package = Path(normalized_packages["bunker_description"])
    meshes = _validate_mesh_records(evidence["meshes"], package)
    contract_meshes = _validate_mesh_records(contract["meshes"], package)
    if meshes != contract_meshes:
        raise LiveContractError("install/live mesh provenance differs")
    if frozenset(item["uri"] for item in meshes) != \
            _load_runtime_mesh_uris(Path(model["path"])):
        raise LiveContractError("provenance meshes differ from runtime URDF")
    plugins = validate_plugin_maps(evidence["plugins"])
    contract_plugins = contract["plugins"]
    if type(contract_plugins) is not list or len(contract_plugins) != 3:
        raise LiveContractError("install plugin provenance differs")
    install_plugin_paths = []
    for index, record in enumerate(contract_plugins):
        _require_exact_keys(record, INSTALL_PLUGIN_KEYS,
                            "install plugin[%d]" % index)
        if record["ldd_status"] != "PASS":
            raise LiveContractError("install plugin dependency check failed")
        install_plugin_paths.append(record["path"])
    if install_plugin_paths != list(PLUGIN_PATHS):
        raise LiveContractError("install plugin paths differ")
    external = _validate_external_snapshot(
        evidence["external_before"], "external-before snapshot")
    return {
        "install_contract_path": str(contract_path),
        "install_contract_sha256": contract_digest,
        "packages": normalized_packages,
        "renderer_sha256": renderer["sha256"],
        "input_sha256": source["sha256"],
        "model_sha256": model["sha256"],
        "mesh_count": len(meshes), "plugin_count": plugins["count"],
        "external_before": external,
    }


def validate_logs(evidence):
    _require_exact_keys(evidence, LOG_KEYS, "logs")
    path = _canonical_regular_file(evidence["launch_path"], "launch log")
    digest = evidence["launch_sha256"]
    if type(digest) is not str or not SHA256_RE.fullmatch(digest):
        raise LiveContractError("launch log hash is malformed")
    if _sha256_file(path) != digest:
        raise LiveContractError("launch log hash differs")
    if evidence["fatal_matches"] != []:
        raise LiveContractError("launch log contains fatal matches")
    return {
        "launch_path": str(path), "launch_sha256": digest,
        "fatal_matches": [],
    }


def validate_shutdown(evidence):
    _require_exact_keys(evidence, SHUTDOWN_KEYS, "shutdown")
    for key in ("probe_status", "launcher_status"):
        if type(evidence[key]) is not int or evidence[key] != 0:
            raise LiveContractError("shutdown %s differs" % key)
    if evidence["cleanup_owner"] != "inner":
        raise LiveContractError("shutdown cleanup owner differs")
    sequence = evidence["signal_sequence"]
    if sequence != ["SIGINT"]:
        raise LiveContractError("shutdown signal sequence differs")
    if type(evidence["escalated"]) is not bool or evidence["escalated"]:
        raise LiveContractError("shutdown escalation is forbidden")
    if evidence["remaining_pids"] != [] or evidence["remaining_ports"] != []:
        raise LiveContractError("shutdown left runtime resources")
    return copy.deepcopy(evidence)


def _validate_process_identity(value, label):
    _require_exact_keys(value, PROCESS_ID_KEYS, label)
    normalized = {}
    for key in sorted(PROCESS_ID_KEYS):
        item = value[key]
        if type(item) is not int or item <= 1:
            raise LiveContractError("%s.%s differs" % (label, key))
        normalized[key] = item
    return normalized


def _validate_processes(evidence):
    _require_exact_keys(evidence, PROBE_SECTION_KEYS["processes"],
                        "processes")
    owned = _validate_process_identity(evidence["owned_root"], "owned root")
    gzserver = evidence["gzserver"]
    _require_exact_keys(gzserver, GZSERVER_KEYS, "gzserver")
    gz_identity = _validate_process_identity(
        {key: gzserver[key] for key in PROCESS_ID_KEYS}, "gzserver identity")
    if (owned["pid"] != owned["pgid"] or owned["pid"] != owned["sid"] or
            gz_identity["pid"] == owned["pid"] or
            gz_identity["pgid"] != owned["pgid"] or
            gz_identity["sid"] != owned["sid"]):
        raise LiveContractError("owned process-group identity differs")
    executable = _canonical_regular_file(
        gzserver["executable"], "gzserver executable")
    if not executable.name.startswith("gzserver-"):
        raise LiveContractError("gzserver executable differs")
    plugins = validate_plugin_maps(gzserver["plugin_maps"])
    if evidence["models"] != ["bunker"]:
        raise LiveContractError("process model set differs")
    if evidence["required_nodes"] != [
            "/gazebo", "/ground/velocity_guard"]:
        raise LiveContractError("required node set differs")
    return {
        "owned_root": owned,
        "gzserver": dict(gz_identity, executable=str(executable),
                         plugin_maps=plugins["records"]),
        "models": ["bunker"],
        "required_nodes": ["/gazebo", "/ground/velocity_guard"],
    }


def _validate_clock_summary(value):
    _require_exact_keys(value, CLOCK_SUMMARY_KEYS, "clock summary")
    count = value["count"]
    if type(count) is not int or count < 3:
        raise LiveContractError("clock summary count differs")
    numbers = {key: _finite_float(value[key], "clock summary." + key)
               for key in CLOCK_SUMMARY_KEYS if key != "count"}
    if (numbers["first_stamp"] <= 0.0 or
            numbers["last_stamp"] <= numbers["first_stamp"] or
            numbers["minimum_period"] <= 0.0 or
            not numbers["minimum_period"] <= numbers["median_period"] <=
            numbers["maximum_period"]):
        raise LiveContractError("clock summary timing differs")
    return dict(value)


def _validate_odometry_summary(value):
    _require_exact_keys(value, ODOM_SUMMARY_KEYS, "odometry summary")
    if type(value["count"]) is not int or value["count"] < 20:
        raise LiveContractError("odometry summary count differs")
    if (value["frame_id"] != "ground/odom" or
            value["child_frame_id"] != "ground/base_link"):
        raise LiveContractError("odometry summary frames differ")
    first = _finite_float(value["first_stamp"], "odometry first stamp")
    last = _finite_float(value["last_stamp"], "odometry last stamp")
    period = _finite_float(value["median_period"], "odometry period")
    minimums = _finite_mapping(
        value["position_min"], VECTOR3_KEYS, "odometry position minimum")
    maximums = _finite_mapping(
        value["position_max"], VECTOR3_KEYS, "odometry position maximum")
    if (first <= 0.0 or last <= first or not 0.016 <= period <= 0.030 or
            any(minimums[key] > maximums[key] for key in VECTOR3_KEYS)):
        raise LiveContractError("odometry summary domain differs")
    for key in ("linear_twist_abs_max", "angular_twist_abs_max",
                "quaternion_norm_min", "quaternion_norm_max"):
        if _finite_float(value[key], "odometry " + key) < 0.0:
            raise LiveContractError("odometry summary extrema differ")
    if (value["quaternion_norm_min"] <= 0.0 or
            value["quaternion_norm_min"] > value["quaternion_norm_max"]):
        raise LiveContractError("odometry quaternion summary differs")
    return copy.deepcopy(value)


def _validate_scan_summary(value):
    _require_exact_keys(value, SCAN_SUMMARY_KEYS, "scan summary")
    if type(value["count"]) is not int or value["count"] < 8:
        raise LiveContractError("scan summary count differs")
    if type(value["positive_infinity_count"]) is not int or \
            value["positive_infinity_count"] < 0:
        raise LiveContractError("scan infinity count differs")
    if value["frame_id"] != SCAN_CONTRACT["frame_id"]:
        raise LiveContractError("scan summary frame differs")
    for key in ("range_count", "intensity_count"):
        if type(value[key]) is not int or value[key] != SCAN_CONTRACT[key]:
            raise LiveContractError("scan summary %s differs" % key)
    first = _finite_float(value["first_stamp"], "scan first stamp")
    last = _finite_float(value["last_stamp"], "scan last stamp")
    period = _finite_float(value["median_period"], "scan median period")
    if first <= 0.0 or last <= first or not 0.050 <= period <= 0.090:
        raise LiveContractError("scan summary timing differs")
    for key in ("angle_min", "angle_max", "angle_increment",
                "range_min", "range_max"):
        observed = _finite_float(value[key], "scan summary." + key)
        if abs(observed - SCAN_CONTRACT[key]) > 1e-6:
            raise LiveContractError("scan summary metadata differs")
    for key in ("time_increment", "scan_time"):
        observed = _finite_float(value[key], "scan summary." + key)
        if abs(observed - SCAN_CONTRACT[key]) > 1e-9:
            raise LiveContractError("scan summary timing metadata differs")
    finite_min = _finite_float(value["finite_range_min"],
                               "scan finite range minimum")
    finite_max = _finite_float(value["finite_range_max"],
                               "scan finite range maximum")
    intensity_min = _finite_float(value["intensity_min"],
                                  "scan intensity minimum")
    intensity_max = _finite_float(value["intensity_max"],
                                  "scan intensity maximum")
    if (not SCAN_CONTRACT["range_min"] - 1e-6 <= finite_min <= finite_max <=
            SCAN_CONTRACT["range_max"] + 1e-6 or
            intensity_min > intensity_max):
        raise LiveContractError("scan summary extrema differ")
    return copy.deepcopy(value)


def _validate_graph_section(graph, processes=None):
    _require_exact_keys(graph, PROBE_SECTION_KEYS["graph"], "graph section")
    snapshot_stamp = _finite_float(
        graph["snapshot_stamp"], "graph snapshot stamp")
    if snapshot_stamp <= 0.0:
        raise LiveContractError("graph snapshot stamp must be nonzero")
    validate_graph({
        "gzserver_count": 1,
        "nodes": (["/gazebo", "/ground/velocity_guard"]
                  if processes is None else processes["required_nodes"]),
        "models": (["bunker"] if processes is None
                   else processes["models"]),
        "topics": graph["topics"],
    })


def _validate_samples_section(samples):
    _require_exact_keys(samples, PROBE_SECTION_KEYS["samples"], "samples")
    clock = _validate_clock_summary(samples["clock"])
    _validate_odometry_summary(samples["odom"])
    _validate_scan_summary(samples["scan"])
    return clock


def _validate_guard_section(guard):
    _require_exact_keys(guard, PROBE_SECTION_KEYS["guard"], "guard section")
    guard_summary = validate_guard_admission(guard["records"])
    if type(guard["probe_disconnected"]) is not bool or \
            guard["probe_disconnected"] is not True:
        raise LiveContractError("guard probe remained connected")
    post_stamp = _finite_float(
        guard["post_probe_snapshot_stamp"], "guard post-probe stamp")
    if post_stamp <= guard_summary["records"][-1]["output_stamp"]:
        raise LiveContractError("guard graph snapshot preceded probes")


def _validate_motion_section(motion):
    _require_exact_keys(motion, PROBE_SECTION_KEYS["motion"], "motion")
    forward = validate_motion_phase("forward", motion["forward"])
    rotation = validate_motion_phase("rotation", motion["rotation"])
    if rotation["baseline"] != forward["final"]:
        raise LiveContractError("motion phases are discontinuous")


def _probe_provenance_payload(payload, result_document):
    provenance = payload["provenance"]
    if result_document:
        return {
            key: provenance[key]
            for key in PROBE_SECTION_KEYS["provenance"]
        }
    return provenance


def _validate_tf_section(tf, provenance_payload, clock):
    _require_exact_keys(tf, PROBE_SECTION_KEYS["tf"], "TF section")
    if tf["forbidden_frames"] != []:
        raise LiveContractError("forbidden TF frames were observed")
    expected = load_expected_fixed_transforms(
        provenance_payload["model"]["path"],
        provenance_payload["model"]["sha256"])
    validate_tf_authorities(tf["edges"], clock["last_stamp"], expected)


def _require_path_parent(value, run_dir, label):
    if Path(value).parent != run_dir:
        raise LiveContractError("%s is not bound to the declared run" % label)


def _validate_completed_sections(payload, result_document=False):
    run_dir = _validate_run_directory(payload["run"]["run_dir"])
    environment = validate_environment(payload["environment"])
    if Path(environment["run_dir"]) != run_dir:
        raise LiveContractError(
            "environment is not bound to the declared run")
    provenance_payload = _probe_provenance_payload(payload, result_document)
    provenance = validate_provenance(provenance_payload)
    _require_path_parent(
        provenance["install_contract_path"], run_dir, "install contract")
    _require_path_parent(
        provenance_payload["model"]["path"], run_dir, "runtime model")
    processes = _validate_processes(payload["processes"])
    _validate_graph_section(payload["graph"], processes)
    clock = _validate_samples_section(payload["samples"])
    _validate_guard_section(payload["guard"])
    _validate_motion_section(payload["motion"])
    _validate_tf_section(payload["tf"], provenance_payload, clock)
    collision = validate_collision(payload["collision"])
    _require_path_parent(
        collision["raw_info_path"], run_dir, "collision raw info")


def _validate_available_failure_sections(payload, result_document=False):
    run_dir = _validate_run_directory(payload["run"]["run_dir"])
    if payload["environment"] is not None:
        environment = validate_environment(payload["environment"])
        if Path(environment["run_dir"]) != run_dir:
            raise LiveContractError(
                "environment is not bound to the declared run")
    provenance_payload = payload["provenance"]
    if provenance_payload is not None and result_document:
        provenance_payload = _probe_provenance_payload(payload, True)
        null_count = sum(value is None for value in provenance_payload.values())
        if null_count not in (0, len(provenance_payload)):
            raise LiveContractError(
                "result provenance is only partially available")
        if null_count == len(provenance_payload):
            provenance_payload = None
    if provenance_payload is not None:
        provenance = validate_provenance(provenance_payload)
        _require_path_parent(
            provenance["install_contract_path"], run_dir,
            "install contract")
        _require_path_parent(
            provenance_payload["model"]["path"], run_dir,
            "runtime model")
    processes = None
    if payload["processes"] is not None:
        processes = _validate_processes(payload["processes"])
    if payload["graph"] is not None:
        _validate_graph_section(payload["graph"], processes)
    clock = None
    if payload["samples"] is not None:
        clock = _validate_samples_section(payload["samples"])
    if payload["guard"] is not None:
        _validate_guard_section(payload["guard"])
    if payload["motion"] is not None:
        _validate_motion_section(payload["motion"])
    if payload["tf"] is not None:
        if provenance_payload is None or clock is None:
            raise LiveContractError(
                "available TF evidence requires provenance and samples")
        _validate_tf_section(payload["tf"], provenance_payload, clock)
    if payload["collision"] is not None:
        collision = validate_collision(payload["collision"])
        _require_path_parent(
            collision["raw_info_path"], run_dir, "collision raw info")


def _contains_null(value):
    if value is None:
        return True
    if type(value) is list:
        return any(_contains_null(item) for item in value)
    if type(value) is dict:
        return any(_contains_null(item) for item in value.values())
    return False


def _normalize_probe_sections(payload):
    environment = payload.get("environment")
    if environment is not None:
        audit = environment["writable_fd_audit"]
        audit["checked_pids"] = sorted(audit["checked_pids"])
        audit["writable_files"] = sorted(audit["writable_files"])
        audit["allowed_kernel_endpoints"] = sorted(
            audit["allowed_kernel_endpoints"])
    provenance = payload.get("provenance")
    if provenance is not None:
        if provenance.get("meshes") is not None:
            provenance["meshes"] = sorted(
                provenance["meshes"], key=lambda item: item["uri"])
        if provenance.get("plugins") is not None:
            provenance["plugins"] = validate_plugin_maps(
                provenance["plugins"])["records"]
    processes = payload.get("processes")
    if processes is not None:
        processes["gzserver"]["plugin_maps"] = validate_plugin_maps(
            processes["gzserver"]["plugin_maps"])["records"]
    graph = payload.get("graph")
    if graph is not None:
        for record in graph["topics"].values():
            record["publishers"] = sorted(record["publishers"])
            record["subscribers"] = sorted(record["subscribers"])
    tf = payload.get("tf")
    if tf is not None:
        tf["edges"] = sorted(
            tf["edges"], key=lambda item: (
                item["parent"], item["child"], item["channel"],
                item["authority"]))
        tf["forbidden_frames"] = sorted(tf["forbidden_frames"])
    return payload


def _validate_run_id(value):
    if type(value) is not str or RUN_ID_RE.fullmatch(value) is None:
        raise LiveContractError("run id is invalid")
    return value


def _validate_utc(value, label):
    if type(value) is not str or UTC_RE.fullmatch(value) is None:
        raise LiveContractError("%s is invalid" % label)
    return value


def _validate_run_directory(value):
    run_dir = _canonical_directory(value, "run directory")
    run_root = _canonical_directory(RUN_ROOT, "run root")
    if not _is_descendant(run_dir, run_root):
        raise LiveContractError("run directory is outside run root")
    if stat.S_IMODE(run_dir.stat().st_mode) != 0o700:
        raise LiveContractError("run directory mode differs")
    return run_dir


def _validate_string_list(value, label, allow_empty=True):
    if (type(value) is not list or
            any(type(item) is not str or not item for item in value)):
        raise LiveContractError("%s must be a string list" % label)
    if not allow_empty and not value:
        raise LiveContractError("%s must not be empty" % label)
    return list(value)


def _validate_probe_run(run):
    _require_exact_keys(run, PROBE_SECTION_KEYS["run"], "probe run")
    run_id = _validate_run_id(run["run_id"])
    run_dir = _validate_run_directory(run["run_dir"])
    started = _validate_utc(run["started_utc"], "probe start time")
    if run["probe_state"] not in ("seed", "completed"):
        raise LiveContractError("probe state differs")
    return {
        "run_id": run_id, "run_dir": str(run_dir),
        "started_utc": started, "probe_state": run["probe_state"],
    }


def _validate_section_shapes(payload, keys):
    for name, expected_keys in keys.items():
        value = payload[name]
        if value is not None:
            _require_exact_keys(value, expected_keys, name)


def make_probe_failure_seed(run_id, run_dir, started_utc, fatal_error):
    if type(fatal_error) is not str or not fatal_error:
        raise LiveContractError("seed fatal error is invalid")
    normalized_run = _validate_probe_run({
        "run_id": run_id, "run_dir": str(run_dir),
        "started_utc": started_utc, "probe_state": "seed",
    })
    payload = {key: None for key in PROBE_KEYS}
    payload.update({
        "schema_version": PROBE_SCHEMA_VERSION,
        "status": "FAIL", "fatal_errors": [fatal_error],
        "run": normalized_run,
    })
    return validate_probe_evidence(payload)


def validate_probe_evidence(payload):
    _require_exact_keys(payload, PROBE_KEYS, "probe evidence")
    if type(payload["schema_version"]) is not int or \
            payload["schema_version"] != PROBE_SCHEMA_VERSION:
        raise LiveContractError("probe schema version differs")
    if payload["status"] not in ("PASS", "FAIL"):
        raise LiveContractError("probe status differs")
    fatal_errors = _validate_string_list(
        payload["fatal_errors"], "probe fatal errors")
    run = _validate_probe_run(payload["run"])
    _validate_section_shapes(
        payload, {key: value for key, value in PROBE_SECTION_KEYS.items()
                  if key != "run"})
    if run["probe_state"] == "seed":
        if payload["status"] != "FAIL" or not fatal_errors:
            raise LiveContractError("probe seed must fail closed")
        for key in PROBE_SECTION_KEYS:
            if key != "run" and payload[key] is not None:
                raise LiveContractError("probe seed section must be null")
    elif payload["status"] == "PASS":
        if fatal_errors:
            raise LiveContractError("passing probe has fatal errors")
        for key in PROBE_SECTION_KEYS:
            if payload[key] is None:
                raise LiveContractError("passing probe has a null section")
        if _contains_null(payload):
            raise LiveContractError("passing probe contains a null value")
        _validate_completed_sections(payload)
    else:
        if not fatal_errors:
            raise LiveContractError("failing probe lacks fatal errors")
        _validate_available_failure_sections(payload)
    normalized = copy.deepcopy(payload)
    normalized["run"] = run
    _normalize_probe_sections(normalized)
    _require_finite_json(normalized, "probe evidence")
    return normalized


def _validate_launcher_run(run):
    _require_exact_keys(run, LAUNCHER_RUN_KEYS, "launcher run")
    run_id = _validate_run_id(run["run_id"])
    run_dir = _validate_run_directory(run["run_dir"])
    started = _validate_utc(run["started_utc"], "launcher start time")
    finished = _validate_utc(run["finished_utc"], "launcher finish time")
    wall_seconds = _finite_float(run["wall_seconds"], "launcher wall seconds")
    supervisor_status = run["supervisor_status"]
    if wall_seconds < 0.0 or type(supervisor_status) is not int or \
            supervisor_status < 0:
        raise LiveContractError("launcher run status differs")
    return {
        "run_id": run_id, "run_dir": str(run_dir),
        "started_utc": started, "finished_utc": finished,
        "wall_seconds": wall_seconds,
        "supervisor_status": supervisor_status,
    }


def _validate_failure_logs(logs):
    _require_exact_keys(logs, LOG_KEYS, "logs")
    fatal_matches = _validate_string_list(logs["fatal_matches"],
                                          "log fatal matches")
    path = logs["launch_path"]
    digest = logs["launch_sha256"]
    if (path is None) != (digest is None):
        raise LiveContractError("failure log path/hash pair differs")
    if path is not None:
        path = str(_canonical_regular_file(path, "launch log"))
        if type(digest) is not str or not SHA256_RE.fullmatch(digest):
            raise LiveContractError("launch log hash is malformed")
        if _sha256_file(Path(path)) != digest:
            raise LiveContractError("launch log hash differs")
    return {
        "launch_path": path, "launch_sha256": digest,
        "fatal_matches": fatal_matches,
    }


def _validate_shutdown_shape(shutdown):
    _require_exact_keys(shutdown, SHUTDOWN_KEYS, "shutdown")
    result = copy.deepcopy(shutdown)
    for key in ("probe_status", "launcher_status"):
        if type(shutdown[key]) is not int or shutdown[key] < 0:
            raise LiveContractError("shutdown %s is invalid" % key)
    if shutdown["cleanup_owner"] not in ("inner", "outer-watchdog"):
        raise LiveContractError("shutdown cleanup owner is invalid")
    result["signal_sequence"] = _validate_string_list(
        shutdown["signal_sequence"], "shutdown signal sequence")
    if type(shutdown["escalated"]) is not bool:
        raise LiveContractError("shutdown escalation flag is invalid")
    result["remaining_pids"] = _validate_pid_list(
        shutdown["remaining_pids"], "remaining PIDs")
    ports = shutdown["remaining_ports"]
    if (type(ports) is not list or any(
            type(port) is not int or not 1 <= port <= 65535
            for port in ports) or len(ports) != len(set(ports))):
        raise LiveContractError("remaining ports differ")
    result["remaining_ports"] = sorted(ports)
    return result


def _validate_launcher(payload):
    _require_exact_keys(payload, LAUNCHER_KEYS, "launcher evidence")
    if type(payload["schema_version"]) is not int or \
            payload["schema_version"] != LAUNCHER_SCHEMA_VERSION:
        raise LiveContractError("launcher schema version differs")
    run = _validate_launcher_run(payload["run"])
    logs = _validate_failure_logs(payload["logs"])
    shutdown = _validate_shutdown_shape(payload["shutdown"])
    if payload["external_after"] is not None and \
            type(payload["external_after"]) is not dict:
        raise LiveContractError("external-after snapshot differs")
    first_failure = payload["first_failure"]
    if first_failure is not None:
        _require_exact_keys(first_failure, FIRST_FAILURE_KEYS,
                            "first failure")
        if (type(first_failure["exit_class"]) is not int or
                first_failure["exit_class"] <= 0 or
                type(first_failure["phase"]) is not str or
                not first_failure["phase"] or
                type(first_failure["message"]) is not str or
                not first_failure["message"]):
            raise LiveContractError("first failure record differs")
    fatal_errors = _validate_string_list(
        payload["fatal_errors"], "launcher fatal errors")
    failure_indicated = (
        run["supervisor_status"] != 0 or
        shutdown["probe_status"] != 0 or
        shutdown["launcher_status"] != 0 or
        shutdown["cleanup_owner"] != "inner" or
        shutdown["signal_sequence"] != ["SIGINT"] or
        shutdown["escalated"] or bool(shutdown["remaining_pids"]) or
        bool(shutdown["remaining_ports"]) or
        logs["launch_path"] is None or logs["launch_sha256"] is None or
        bool(logs["fatal_matches"]) or payload["external_after"] is None or
        first_failure is not None or bool(fatal_errors))
    if failure_indicated:
        if (first_failure is None or not fatal_errors or
                fatal_errors[0] != first_failure["message"]):
            raise LiveContractError(
                "launcher failure lacks its ordered first failure")
    elif first_failure is not None or fatal_errors:
        raise LiveContractError("successful launcher reports a failure")
    normalized = copy.deepcopy(payload)
    normalized["run"] = run
    normalized["logs"] = logs
    normalized["shutdown"] = shutdown
    normalized["fatal_errors"] = fatal_errors
    _require_finite_json(normalized, "launcher evidence")
    return normalized


def _append_unique(values, item):
    if item not in values:
        values.append(item)


def merge_result(probe_payload, launcher_payload):
    probe = validate_probe_evidence(probe_payload)
    launcher = _validate_launcher(launcher_payload)
    for key in ("run_id", "run_dir", "started_utc"):
        if probe["run"][key] != launcher["run"][key]:
            raise LiveContractError("probe/launcher run identity differs")
    result = copy.deepcopy(probe)
    result["schema_version"] = RESULT_SCHEMA_VERSION
    result["run"] = copy.deepcopy(launcher["run"])
    result["logs"] = copy.deepcopy(launcher["logs"])
    result["shutdown"] = copy.deepcopy(launcher["shutdown"])
    provenance = ({key: None for key in PROBE_SECTION_KEYS["provenance"]}
                  if probe["provenance"] is None
                  else copy.deepcopy(probe["provenance"]))
    external_before = provenance.get("external_before")
    external_after = copy.deepcopy(launcher["external_after"])
    provenance["external_after"] = external_after
    provenance["external_unchanged"] = (
        external_before is not None and external_after is not None and
        external_before == external_after)
    result["provenance"] = provenance
    fatal_errors = []
    first_failure = launcher["first_failure"]
    if first_failure is not None:
        _append_unique(fatal_errors, first_failure["message"])
    for error in launcher["fatal_errors"]:
        _append_unique(fatal_errors, error)
    if probe["run"]["probe_state"] != "seed":
        for error in probe["fatal_errors"]:
            _append_unique(fatal_errors, error)
    if launcher["run"]["supervisor_status"] != 0:
        _append_unique(fatal_errors, "supervisor returned nonzero status")
    if launcher["shutdown"]["probe_status"] != 0:
        _append_unique(fatal_errors, "probe returned nonzero status")
    if launcher["shutdown"]["launcher_status"] != 0:
        _append_unique(fatal_errors, "launcher returned nonzero status")
    if launcher["shutdown"]["cleanup_owner"] != "inner":
        _append_unique(
            fatal_errors, "cleanup owner is " +
            launcher["shutdown"]["cleanup_owner"])
    if launcher["shutdown"]["signal_sequence"] != ["SIGINT"]:
        _append_unique(fatal_errors, "shutdown signal sequence differs")
    if launcher["shutdown"]["escalated"]:
        _append_unique(fatal_errors, "shutdown required escalation")
    if launcher["shutdown"]["remaining_pids"]:
        _append_unique(fatal_errors, "shutdown left remaining PIDs")
    if launcher["shutdown"]["remaining_ports"]:
        _append_unique(fatal_errors, "shutdown left remaining ports")
    if (launcher["logs"]["launch_path"] is None or
            launcher["logs"]["launch_sha256"] is None):
        _append_unique(fatal_errors, "launch log is unavailable")
    for match in launcher["logs"]["fatal_matches"]:
        _append_unique(fatal_errors, "launch log: " + match)
    if (probe["run"]["probe_state"] == "completed" and
            (external_before is None or external_after is None or
             external_before != external_after)):
        _append_unique(fatal_errors, "external repository state changed")
    pass_candidate = (
        probe["run"]["probe_state"] == "completed" and
        probe["status"] == "PASS" and not fatal_errors and
        launcher["run"]["supervisor_status"] == 0 and
        launcher["shutdown"]["probe_status"] == 0 and
        launcher["shutdown"]["launcher_status"] == 0)
    if pass_candidate:
        result["status"] = "PASS"
        result["fatal_errors"] = []
    else:
        result["status"] = "FAIL"
        if not fatal_errors:
            fatal_errors.append("runtime evidence failed")
        result["fatal_errors"] = fatal_errors
    return validate_live_evidence(result)


def _validate_result_run(run):
    return _validate_launcher_run(run)


def validate_live_evidence(payload):
    _require_exact_keys(payload, RESULT_KEYS, "live evidence")
    if type(payload["schema_version"]) is not int or \
            payload["schema_version"] != RESULT_SCHEMA_VERSION:
        raise LiveContractError("result schema version differs")
    if payload["status"] not in ("PASS", "FAIL"):
        raise LiveContractError("result status differs")
    fatal_errors = _validate_string_list(
        payload["fatal_errors"], "result fatal errors")
    run = _validate_result_run(payload["run"])
    _validate_section_shapes(
        payload, {key: value for key, value in PROBE_SECTION_KEYS.items()
                  if key not in ("run", "provenance")})
    provenance = payload["provenance"]
    _require_exact_keys(provenance, RESULT_PROVENANCE_KEYS,
                        "result provenance")
    external_before = provenance["external_before"]
    external_after = provenance["external_after"]
    if external_before is not None:
        external_before = _validate_external_snapshot(
            external_before, "external-before snapshot", require_clean=False)
    if external_after is not None:
        external_after = _validate_external_snapshot(
            external_after, "external-after snapshot", require_clean=False)
    if type(provenance["external_unchanged"]) is not bool:
        raise LiveContractError("external unchanged flag is invalid")
    if provenance["external_unchanged"] and (
            external_before is None or external_after is None or
            external_before != external_after):
        raise LiveContractError("external unchanged claim differs")
    logs = _validate_failure_logs(payload["logs"])
    shutdown = _validate_shutdown_shape(payload["shutdown"])
    run_dir = Path(run["run_dir"])
    if logs["launch_path"] is not None:
        _require_path_parent(logs["launch_path"], run_dir, "launch log")
    if payload["status"] == "PASS":
        if fatal_errors:
            raise LiveContractError("passing result has fatal errors")
        if _contains_null(payload):
            raise LiveContractError("passing result contains a null value")
        if run["supervisor_status"] != 0:
            raise LiveContractError("passing supervisor status differs")
        validate_logs(logs)
        validate_shutdown(shutdown)
        external_before = _validate_external_snapshot(
            provenance["external_before"], "external-before snapshot")
        external_after = _validate_external_snapshot(
            provenance["external_after"], "external-after snapshot")
        if (provenance["external_unchanged"] is not True or
                external_before != external_after):
            raise LiveContractError("passing result changed external inputs")
        _validate_completed_sections(payload, result_document=True)
    else:
        if not fatal_errors:
            raise LiveContractError("failing result lacks fatal errors")
        _validate_available_failure_sections(payload, result_document=True)
    normalized = copy.deepcopy(payload)
    normalized["run"] = run
    normalized["fatal_errors"] = fatal_errors
    normalized["logs"] = logs
    normalized["shutdown"] = shutdown
    _normalize_probe_sections(normalized)
    _require_finite_json(normalized, "live evidence")
    return normalized


def _atomic_write_document(path, payload, expected_basename,
                           declared_run_dir):
    if not isinstance(path, (str, os.PathLike)):
        raise LiveContractError("output path is invalid")
    candidate = Path(path)
    if not candidate.is_absolute() or candidate.name != expected_basename:
        raise LiveContractError("output path or basename differs")
    parent = _validate_run_directory(candidate.parent)
    expected_run_dir = _validate_run_directory(declared_run_dir)
    if parent != expected_run_dir:
        raise LiveContractError(
            "output destination differs from the document run directory")
    expected = parent / expected_basename
    if candidate != expected or candidate.is_symlink():
        raise LiveContractError("output path must be canonical")
    try:
        serialized = json.dumps(
            payload, allow_nan=False, sort_keys=True,
            separators=(",", ":")) + "\n"
    except (TypeError, ValueError) as error:
        raise LiveContractError("document is not strict JSON: %s" % error)
    descriptor = None
    temporary_path = None
    try:
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=".%s." % expected_basename, suffix=".tmp",
            dir=str(parent))
        temporary_path = Path(temporary_name)
        os.fchmod(descriptor, 0o600)
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            descriptor = None
            stream.write(serialized)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(str(temporary_path), str(expected))
        temporary_path = None
        directory_fd = os.open(str(parent), os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    except OSError as error:
        raise LiveContractError("atomic write failed: %s" % error)
    finally:
        if descriptor is not None:
            os.close(descriptor)
        if temporary_path is not None:
            try:
                temporary_path.unlink()
            except FileNotFoundError:
                pass
    return expected


def atomic_write_probe(path, payload):
    normalized = validate_probe_evidence(payload)
    return _atomic_write_document(
        path, normalized, "probe-evidence.json",
        normalized["run"]["run_dir"])


def atomic_write_result(path, payload):
    normalized = validate_live_evidence(payload)
    return _atomic_write_document(
        path, normalized, "result.json", normalized["run"]["run_dir"])


def _strict_object_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise LiveContractError("JSON object contains duplicate key: " + key)
        result[key] = value
    return result


def _load_json_document(path, label):
    file_path = _canonical_regular_file(path, label)
    try:
        with file_path.open("r", encoding="utf-8") as stream:
            return json.load(
                stream,
                object_pairs_hook=_strict_object_pairs,
                parse_constant=lambda value: (_ for _ in ()).throw(
                    ValueError("non-finite token " + value)))
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as error:
        raise LiveContractError("cannot read %s: %s" % (label, error))


def _build_argument_parser():
    parser = _ContractArgumentParser(prog="bunker_live_contract.py")
    commands = parser.add_subparsers(dest="command", required=True)
    seed = commands.add_parser("seed-probe")
    seed.add_argument("--run-id", required=True)
    seed.add_argument("--run-dir", required=True)
    seed.add_argument("--started-utc", required=True)
    seed.add_argument("--fatal-error", required=True)
    seed.add_argument("--output", required=True)
    validate_probe = commands.add_parser("validate-probe")
    validate_probe.add_argument("--input", required=True)
    finalize = commands.add_parser("finalize")
    finalize.add_argument("--probe", required=True)
    finalize.add_argument("--launcher", required=True)
    finalize.add_argument("--output", required=True)
    validate_result = commands.add_parser("validate-result")
    validate_result.add_argument("--input", required=True)
    return parser


def main(argv=None):
    try:
        arguments = _build_argument_parser().parse_args(argv)
        if arguments.command == "seed-probe":
            payload = make_probe_failure_seed(
                arguments.run_id, arguments.run_dir,
                arguments.started_utc, arguments.fatal_error)
            atomic_write_probe(arguments.output, payload)
            return 0
        if arguments.command == "validate-probe":
            payload = validate_probe_evidence(
                _load_json_document(arguments.input, "probe evidence"))
            return 0 if payload["status"] == "PASS" else 1
        if arguments.command == "finalize":
            result = merge_result(
                _load_json_document(arguments.probe, "probe evidence"),
                _load_json_document(arguments.launcher, "launcher evidence"))
            atomic_write_result(arguments.output, result)
            return 0 if result["status"] == "PASS" else 1
        result = validate_live_evidence(
            _load_json_document(arguments.input, "result evidence"))
        return 0 if result["status"] == "PASS" else 1
    except _UsageError as error:
        print("BUNKER live contract usage: %s" % error, file=sys.stderr)
        return 64
    except LiveContractError as error:
        print("BUNKER live contract: %s" % error, file=sys.stderr)
        return 65


if __name__ == "__main__":
    sys.exit(main())
