import copy
import contextlib
import hashlib
import io
import json
import math
import os
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from tools import bunker_live_contract as live


ROOT = Path(__file__).resolve().parents[1]
TEMP_ROOT = ROOT / "logs/bunker_standalone/engineering-tmp"
PLUGIN_PATHS = (
    "/opt/ros/noetic/lib/libgazebo_ros_planar_move.so",
    "/opt/ros/noetic/lib/libgazebo_ros_laser.so",
    "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins/libRayPlugin.so",
)


def _vector(x=0.0, y=0.0, z=0.0):
    return {"x": x, "y": y, "z": z}


def _quaternion(x=0.0, y=0.0, z=0.0, w=1.0):
    return {"x": x, "y": y, "z": z, "w": w}


def _odom_samples(count=20, period=0.02):
    return [{
        "stamp": 1.0 + index * period,
        "frame_id": "ground/odom",
        "child_frame_id": "ground/base_link",
        "position": _vector(0.01 * index, 0.0, 0.36),
        "orientation": _quaternion(),
        "linear_twist": _vector(0.25, 0.0, 0.0),
        "angular_twist": _vector(),
    } for index in range(count)]


def _scan_samples(count=8, period=0.066):
    records = []
    for index in range(count):
        ranges = [float("inf")] * 720
        ranges[360] = 1.5 + 0.01 * index
        records.append({
            "stamp": 2.0 + index * period,
            "frame_id": "ground/lidar_2d_link",
            "ranges": ranges,
            "intensities": [0.0] * 720,
            "angle_min": -math.pi,
            "angle_max": math.pi,
            "angle_increment": 2.0 * math.pi / 719.0,
            "range_min": 0.12,
            "range_max": 8.0,
            "time_increment": 0.0,
            "scan_time": 0.0,
        })
    return records


def _guard_records():
    return [
        {
            "case": "over_limit",
            "input_tokens": ["0.75", "0", "0", "0", "0", "2.0"],
            "output": [0.5, 0.0, 0.0, 0.0, 0.0, 1.0],
            "input_stamp": 10.0,
            "output_stamp": 10.04,
            "latency": 0.04,
        },
        {
            "case": "nonplanar",
            "input_tokens": ["0", "0.1", "0", "0", "0", "0"],
            "output": [0.0] * 6,
            "input_stamp": 11.0,
            "output_stamp": 11.04,
            "latency": 0.04,
        },
        {
            "case": "nonfinite",
            "input_tokens": ["nan", "0", "0", "0", "0", "0"],
            "output": [0.0] * 6,
            "input_stamp": 12.0,
            "output_stamp": 12.04,
            "latency": 0.04,
        },
    ]


def _pose(model_x, model_y, model_yaw, odom_x=None, odom_y=None,
          odom_yaw=None):
    return {
        "model": {
            "x": model_x, "y": model_y, "z": 0.36,
            "roll": 0.0, "pitch": 0.0, "yaw": model_yaw,
        },
        "odom": {
            "x": model_x if odom_x is None else odom_x,
            "y": model_y if odom_y is None else odom_y,
            "yaw": model_yaw if odom_yaw is None else odom_yaw,
        },
    }


def _motion(kind):
    if kind == "forward":
        start, target, count = 10.0, 2.0, 40
        baseline = _pose(0.0, 0.0, 0.0)
        final = _pose(0.60, 0.02, 0.01, 0.59, 0.01, 0.0)
        command = (0.25, 0.0)
        metrics = {
            "model_forward": 0.60, "model_lateral": 0.02,
            "odom_forward": 0.59, "odom_lateral": 0.01,
            "model_odom_xy": math.hypot(0.01, 0.01),
            "model_odom_yaw": 0.01,
            "height_change": 0.0, "roll": 0.0, "pitch": 0.0,
        }
    else:
        start, target, count = 20.0, 1.5, 30
        baseline = _pose(0.60, 0.02, 0.01, 0.59, 0.01, 0.0)
        final = _pose(0.62, 0.02, 1.01, 0.61, 0.01, 1.0)
        command = (0.0, 0.5)
        metrics = {
            "model_yaw": 1.0, "model_planar_drift": 0.02,
            "odom_yaw": 1.0,
            "model_odom_xy": math.hypot(0.01, 0.01),
            "model_odom_yaw": 0.01,
            "height_change": 0.0, "roll": 0.0, "pitch": 0.0,
        }
    actual = (count - 1) * 0.05
    last = start + actual
    end = start + target + 0.02
    stop = last + 0.55
    return {
        "kind": kind,
        "command": {
            "linear_x": command[0], "angular_z": command[1],
            "rate_hz": 20.0, "target_duration": target,
            "start_stamp": start, "last_publish_stamp": last,
            "end_stamp": end, "published_count": count,
            "actual_duration": actual, "minimum_period": 0.05,
            "median_period": 0.05, "maximum_period": 0.05,
        },
        "baseline": baseline,
        "final": final,
        "watchdog": {
            "last_command_stamp": last, "stop_observed_stamp": stop,
            "clock_coast": 0.55, "time_to_stop": 0.55,
            "model_linear_speed": 0.0, "model_angular_speed": 0.0,
            "odom_linear_speed": 0.0, "odom_angular_speed": 0.0,
        },
        "stationary": {
            "start_stamp": stop, "end_stamp": stop + 1.0,
            "duration": 1.0, "position_drift": 0.0, "yaw_drift": 0.0,
        },
        "metrics": metrics,
    }


def _graph():
    return {
        "gzserver_count": 1,
        "nodes": ["/gazebo", "/ground/velocity_guard"],
        "models": ["bunker"],
        "topics": {
            name: {
                "type": record["type"],
                "publishers": sorted(record["publishers"]),
                "subscribers": sorted(record["subscribers"]),
            }
            for name, record in live.EXPECTED_GRAPH.items()
        },
    }


def _make_fixed_urdf(path):
    root = ET.Element("robot", {"name": "fixture"})
    ET.SubElement(root, "link", {"name": "base_link"})
    for index, child in enumerate(live.EXPECTED_FIXED_CHILDREN):
        local = child[len("ground/"):]
        ET.SubElement(root, "link", {"name": local})
        joint = ET.SubElement(
            root, "joint", {"name": "joint_%02d" % index, "type": "fixed"})
        xyz = "-0.30 0 0.25" if local == "lidar_2d_link" else "0 0 0"
        ET.SubElement(joint, "origin", {"xyz": xyz, "rpy": "0 0 0"})
        ET.SubElement(joint, "parent", {"link": "base_link"})
        ET.SubElement(joint, "child", {"link": local})
    path.write_bytes(ET.tostring(root, encoding="utf-8") + b"\n")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _render_runtime_urdf(path):
    install = ROOT / "install/p450-clean"
    renderer = (install / "share/bunker_sim_runtime/scripts/"
                "render_bunker_runtime.py")
    source = (install / "share/bunker_description/urdf/"
              "bunker.urdf.xacro")
    environment = dict(os.environ)
    environment["PYTHONPATH"] = (
        str(install / "lib/python3/dist-packages") + ":" +
        "/opt/ros/noetic/lib/python3/dist-packages")
    completed = subprocess.run(
        [str(renderer), str(source)], check=True, stdout=subprocess.PIPE,
        stderr=subprocess.PIPE, env=environment)
    path.write_bytes(completed.stdout)
    path.chmod(0o600)
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _tf_edges(expected, clock=50.0):
    edges = [{
        "parent": "world", "child": "ground/odom",
        "channel": "tf_static", "authority": "/ground/world_to_odom",
        "stamp": 0.0, "translation": [0.0, 0.0, 0.0],
        "rotation": [0.0, 0.0, 0.0, 1.0],
    }, {
        "parent": "ground/odom", "child": "ground/base_link",
        "channel": "tf", "authority": "/gazebo",
        "stamp": clock - 0.02, "translation": [0.1, 0.0, 0.36],
        "rotation": [0.0, 0.0, 0.0, 1.0],
    }]
    for key, transform in sorted(expected.items()):
        edges.append({
            "parent": key[0], "child": key[1], "channel": key[2],
            "authority": "/ground/robot_state_publisher", "stamp": 0.0,
            "translation": list(transform["translation"]),
            "rotation": list(transform["rotation"]),
        })
    return edges


class LiveTestCase(unittest.TestCase):
    def setUp(self):
        TEMP_ROOT.mkdir(mode=0o700, parents=True, exist_ok=True)
        TEMP_ROOT.chmod(0o700)
        self.temporary = tempfile.TemporaryDirectory(dir=str(TEMP_ROOT))
        self.addCleanup(self.temporary.cleanup)
        self.run_dir = Path(self.temporary.name)
        self.run_dir.chmod(0o700)


class LivePrimitiveTest(LiveTestCase):
    def test_exact_keys_and_finite_numbers(self):
        self.assertEqual({"a": 1}, live._require_exact_keys(
            {"a": 1}, frozenset({"a"}), "fixture"))
        for value in (1, 1.25, -3.0):
            self.assertEqual(float(value), live._finite_float(value, "value"))
        for value in (True, False, float("nan"), float("inf"),
                      10 ** 10000, "1", None):
            with self.subTest(value=value):
                with self.assertRaises(live.LiveContractError):
                    live._finite_float(value, "value")
        for value in ({}, {"a": 1, "b": 2}, {"b": 1}, []):
            with self.assertRaises(live.LiveContractError):
                live._require_exact_keys(value, frozenset({"a"}), "fixture")


class LiveRateTest(LiveTestCase):
    def test_median_period_boundaries(self):
        self.assertAlmostEqual(0.02, live.median_period(
            [1.0 + 0.02 * index for index in range(20)], 20))
        for stamps, count in (
                ([1.0], 2), ([1.0, 1.0], 2),
                ([1.0, 0.9], 2), ([1.0, float("nan")], 2),
                ([-1.7e308, 0.0, 1.7e308], 3),
                ([True, 2.0], 2)):
            with self.assertRaises(live.LiveContractError):
                live.median_period(stamps, count)


class LiveClockTest(LiveTestCase):
    def test_clock_is_finite_nonzero_and_increasing(self):
        summary = live.validate_clock_samples([1.0, 1.1, 1.2])
        self.assertEqual(3, summary["count"])
        self.assertGreater(summary["last_stamp"], summary["first_stamp"])
        for stamps in ([0.0, 0.1, 0.2], [1.0, 1.0, 1.1],
                       [1.0, float("inf"), 2.0]):
            with self.assertRaises(live.LiveContractError):
                live.validate_clock_samples(stamps)


class LiveOdomTest(LiveTestCase):
    def test_exact_frames_count_and_rate(self):
        summary = live.validate_odometry_samples(_odom_samples())
        self.assertEqual(20, summary["count"])
        self.assertEqual("ground/odom", summary["frame_id"])
        self.assertEqual("ground/base_link", summary["child_frame_id"])
        for mutation in ("count", "frame", "extra", "rate"):
            samples = _odom_samples()
            if mutation == "count":
                samples.pop()
            elif mutation == "frame":
                samples[0]["frame_id"] = "odom"
            elif mutation == "extra":
                samples[0]["extra"] = 1
            else:
                samples = _odom_samples(period=0.04)
            with self.subTest(mutation=mutation):
                with self.assertRaises(live.LiveContractError):
                    live.validate_odometry_samples(samples)

    def test_pose_and_twist_reject_nonfinite(self):
        for field, component, value in (
                ("position", "x", float("nan")),
                ("orientation", "w", float("inf")),
                ("linear_twist", "y", float("nan")),
                ("angular_twist", "z", float("inf"))):
            samples = _odom_samples()
            samples[0][field][component] = value
            with self.subTest(field=field):
                with self.assertRaises(live.LiveContractError):
                    live.validate_odometry_samples(samples)
        samples = _odom_samples()
        samples[0]["orientation"] = _quaternion(0, 0, 0, 0)
        with self.assertRaises(live.LiveContractError):
            live.validate_odometry_samples(samples)
        samples = _odom_samples()
        samples[0]["orientation"] = _quaternion(
            1e308, 1e308, 1e308, 1e308)
        with self.assertRaises(live.LiveContractError):
            live.validate_odometry_samples(samples)


class LiveScanTest(LiveTestCase):
    def test_exact_metadata_and_lengths(self):
        summary = live.validate_scan_samples(_scan_samples())
        self.assertEqual(8, summary["count"])
        self.assertEqual(720, summary["range_count"])
        for mutation in ("frame", "ranges", "intensities", "metadata", "extra"):
            samples = _scan_samples()
            if mutation == "frame":
                samples[0]["frame_id"] = "lidar_2d_link"
            elif mutation == "ranges":
                samples[0]["ranges"].pop()
            elif mutation == "intensities":
                samples[0]["intensities"].pop()
            elif mutation == "metadata":
                samples[0]["angle_min"] += 1e-3
            else:
                samples[0]["extra"] = 1
            with self.subTest(mutation=mutation):
                with self.assertRaises(live.LiveContractError):
                    live.validate_scan_samples(samples)

    def test_range_and_intensity_domain(self):
        summary = live.validate_scan_samples(_scan_samples())
        self.assertGreater(summary["positive_infinity_count"], 0)
        self.assertTrue(math.isfinite(summary["finite_range_min"]))
        self.assertNotIn("ranges", summary)
        for field, value in (
                ("ranges", float("nan")),
                ("ranges", float("-inf")),
                ("ranges", 0.10), ("ranges", 8.1),
                ("ranges", 10 ** 10000),
                ("intensities", float("inf"))):
            samples = _scan_samples()
            samples[0][field][0] = value
            with self.subTest(field=field, value=value):
                with self.assertRaises(live.LiveContractError):
                    live.validate_scan_samples(samples)


class LiveGuardTest(LiveTestCase):
    def test_three_ordered_cases(self):
        summary = live.validate_guard_admission(_guard_records())
        self.assertEqual(
            ["over_limit", "nonplanar", "nonfinite"], summary["cases"])
        self.assertLessEqual(summary["maximum_latency"], 0.10)
        for mutation in ("order", "output", "latency", "extra"):
            records = _guard_records()
            if mutation == "order":
                records.reverse()
            elif mutation == "output":
                records[0]["output"][0] = 0.49
            elif mutation == "latency":
                records[1]["latency"] = 0.2
            else:
                records[0]["extra"] = 1
            with self.subTest(mutation=mutation):
                with self.assertRaises(live.LiveContractError):
                    live.validate_guard_admission(records)

    def test_nonfinite_input_has_json_token_encoding(self):
        records = _guard_records()
        normalized = live.validate_guard_admission(records)
        serialized = json.dumps(normalized, allow_nan=False, sort_keys=True)
        self.assertIn('"nan"', serialized)
        records[2]["input_tokens"][0] = float("nan")
        with self.assertRaises(live.LiveContractError):
            live.validate_guard_admission(records)


class LiveMotionMathTest(LiveTestCase):
    def test_unwrap_and_local_projection(self):
        self.assertAlmostEqual(0.2, live.unwrap_yaw(math.pi - 0.1,
                                                    -math.pi + 0.1))
        self.assertAlmostEqual(-0.2, live.unwrap_yaw(-math.pi + 0.1,
                                                     math.pi - 0.1))
        projected = live.project_local_delta(
            {"x": 1.0, "y": 2.0, "yaw": math.pi / 2.0},
            {"x": 1.0, "y": 3.0, "yaw": math.pi})
        self.assertAlmostEqual(1.0, projected["forward"])
        self.assertAlmostEqual(0.0, projected["lateral"], places=9)
        self.assertAlmostEqual(math.pi / 2.0, projected["yaw"])
        with self.assertRaises(live.LiveContractError):
            live.unwrap_yaw(-1.7e308, 1.7e308)
        with self.assertRaises(live.LiveContractError):
            live.project_local_delta(
                {"x": -1.7e308, "y": 0.0, "yaw": 0.0},
                {"x": 1.7e308, "y": 0.0, "yaw": 0.0})


class LiveForwardTest(LiveTestCase):
    def test_forward_limits_and_xy_agreement(self):
        summary = live.validate_motion_phase("forward", _motion("forward"))
        self.assertEqual("forward", summary["kind"])
        for key, value in (
                ("model_forward", 0.49), ("model_lateral", 0.11),
                ("model_odom_xy", 0.031)):
            evidence = _motion("forward")
            evidence["metrics"][key] = value
            with self.subTest(key=key):
                with self.assertRaises(live.LiveContractError):
                    live.validate_motion_phase("forward", evidence)
        for field, value in (("z", 0.40), ("roll", 0.06), ("pitch", -0.06)):
            evidence = _motion("forward")
            evidence["final"]["model"][field] = value
            with self.subTest(derived_model_field=field):
                with self.assertRaises(live.LiveContractError):
                    live.validate_motion_phase("forward", evidence)


class LiveWatchdogTest(LiveTestCase):
    def test_sim_time_stop_and_stationary_window(self):
        for mutation in ("coast", "speed", "stationary", "order", "burst"):
            evidence = _motion("forward")
            if mutation == "coast":
                evidence["watchdog"]["clock_coast"] = 0.49
            elif mutation == "speed":
                evidence["watchdog"]["model_linear_speed"] = 0.011
            elif mutation == "stationary":
                evidence["stationary"]["duration"] = 0.99
            elif mutation == "order":
                evidence["stationary"]["start_stamp"] = (
                    evidence["watchdog"]["stop_observed_stamp"] - 0.01)
            else:
                evidence["command"]["minimum_period"] = 0.0
            with self.subTest(mutation=mutation):
                with self.assertRaises(live.LiveContractError):
                    live.validate_motion_phase("forward", evidence)


class LiveRotationTest(LiveTestCase):
    def test_yaw_drift_and_odom_agreement(self):
        summary = live.validate_motion_phase("rotation", _motion("rotation"))
        self.assertEqual("rotation", summary["kind"])
        for key, value in (
                ("model_yaw", 0.84), ("model_planar_drift", 0.081),
                ("model_odom_yaw", 0.041)):
            evidence = _motion("rotation")
            evidence["metrics"][key] = value
            with self.subTest(key=key):
                with self.assertRaises(live.LiveContractError):
                    live.validate_motion_phase("rotation", evidence)


class LiveGraphTest(LiveTestCase):
    def test_exact_topic_types_and_owners(self):
        summary = live.validate_graph(_graph())
        self.assertEqual(4, summary["topic_count"])
        for mutation in ("topic", "owner", "node", "model", "gzserver"):
            evidence = _graph()
            if mutation == "topic":
                evidence["topics"].pop("/ground/scan")
            elif mutation == "owner":
                evidence["topics"]["/ground/odom"]["publishers"] = []
            elif mutation == "node":
                evidence["nodes"].append("/other")
            elif mutation == "model":
                evidence["models"].append("other")
            else:
                evidence["gzserver_count"] = 2
            with self.subTest(mutation=mutation):
                with self.assertRaises(live.LiveContractError):
                    live.validate_graph(evidence)


class LiveTfTest(LiveTestCase):
    def test_complete_nineteen_edge_tree(self):
        urdf = self.run_dir / "rendered-bunker.urdf"
        digest = _render_runtime_urdf(urdf)
        expected = live.load_expected_fixed_transforms(urdf, digest)
        self.assertEqual(17, len(expected))
        summary = live.validate_tf_authorities(
            _tf_edges(expected), 50.0, expected)
        self.assertEqual(19, summary["edge_count"])
        for mutation in ("missing", "authority", "stale", "double", "numeric"):
            edges = _tf_edges(expected)
            if mutation == "missing":
                edges.pop()
            elif mutation == "authority":
                edges[-1]["authority"] = "/other"
            elif mutation == "stale":
                edges[1]["stamp"] = 49.8
            elif mutation == "double":
                edges[-1]["child"] = "ground/ground/wheel4_3_Link"
            else:
                edges[-1]["translation"][0] = 0.01
            with self.subTest(mutation=mutation):
                with self.assertRaises(live.LiveContractError):
                    live.validate_tf_authorities(edges, 50.0, expected)

    def test_oracle_rejects_caller_selected_urdf_hash(self):
        arbitrary = self.run_dir / "arbitrary.urdf"
        digest = _make_fixed_urdf(arbitrary)
        arbitrary.chmod(0o600)
        with self.assertRaises(live.LiveContractError):
            live.load_expected_fixed_transforms(arbitrary, digest)


class LiveBoundaryTest(LiveTestCase):
    def _environment(self):
        names = {
            "TMPDIR": "tmp", "ROS_HOME": "ros-home",
            "ROS_LOG_DIR": "ros-log", "GAZEBO_LOG_PATH": "gazebo-log",
            "IGN_FUEL_CACHE_PATH": "ign-fuel-cache",
            "XDG_CACHE_HOME": "xdg-cache", "XDG_CONFIG_HOME": "xdg-config",
            "XDG_DATA_HOME": "xdg-data",
        }
        for name in names.values():
            (self.run_dir / name).mkdir(mode=0o700, exist_ok=True)
        install = str(ROOT / "install/p450-clean")
        return {
            "home": str(Path.home().resolve()),
            "ros_master_uri": "http://127.0.0.1:11371/",
            "gazebo_master_uri": "http://127.0.0.1:11372",
            "gazebo_model_database_uri": "",
            "search_paths": {
                "ROS_PACKAGE_PATH": install + "/share:/opt/ros/noetic/share",
                "CMAKE_PREFIX_PATH": install + ":/opt/ros/noetic",
                "PYTHONPATH": install + "/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages",
                "LD_LIBRARY_PATH": install + "/lib:/opt/ros/noetic/lib:/opt/ros/noetic/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins",
                "GAZEBO_PLUGIN_PATH": "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:/opt/ros/noetic/lib",
                "GAZEBO_MODEL_PATH": "/usr/share/gazebo-11/models",
                "GAZEBO_RESOURCE_PATH": "/usr/share/gazebo-11",
                "PKG_CONFIG_PATH": install + "/lib/pkgconfig:/opt/ros/noetic/lib/pkgconfig:/opt/ros/noetic/lib/x86_64-linux-gnu/pkgconfig",
                "OGRE_RESOURCE_PATH": "/usr/lib/x86_64-linux-gnu/OGRE-1.9.0",
            },
            "fixed_environment": dict(live.EXPECTED_FIXED_ENVIRONMENT),
            "state_paths": {
                variable: str(self.run_dir / name)
                for variable, name in names.items()
            },
            "writable_fd_audit": {
                "checked_pids": [123], "writable_files": [],
                "allowed_kernel_endpoints": ["pipe"],
            },
        }

    def test_plugins_collision_environment_and_shutdown(self):
        plugins = [{"basename": Path(path).name, "path": path}
                   for path in PLUGIN_PATHS]
        self.assertEqual(3, live.validate_plugin_maps(plugins)["count"])
        shadowed = copy.deepcopy(plugins)
        shadowed.append(dict(shadowed[0]))
        with self.assertRaises(live.LiveContractError):
            live.validate_plugin_maps(shadowed)

        raw = self.run_dir / "gz-model-info.txt"
        raw.write_text("fixture\n", encoding="utf-8")
        raw_sha = hashlib.sha256(raw.read_bytes()).hexdigest()
        collision = {
            "model": "bunker", "link": "base_link",
            "collisions": [{
                "name": "base_link_collision", "link": "base_link",
                "geometry": "box",
                "origin": [0.018058912, 0.001357451, -0.160420741],
                "size": [1.026335219, 0.782744936, 0.395154782],
            }],
            "mesh_collision_count": 0, "baseline_height": 0.36,
            "final_height": 0.36, "roll": 0.0, "pitch": 0.0,
            "raw_info_path": str(raw), "raw_info_sha256": raw_sha,
        }
        self.assertEqual("proxy-only", live.validate_collision(
            collision)["claim"])
        self.assertEqual(
            self.run_dir, Path(live.validate_environment(
                self._environment())["run_dir"]))
        shutdown = {
            "probe_status": 0, "launcher_status": 0,
            "cleanup_owner": "inner", "signal_sequence": ["SIGINT"],
            "escalated": False, "remaining_pids": [], "remaining_ports": [],
        }
        self.assertEqual("inner", live.validate_shutdown(
            shutdown)["cleanup_owner"])
        bad = dict(shutdown, escalated=True)
        with self.assertRaises(live.LiveContractError):
            live.validate_shutdown(bad)


class LiveProvenanceTest(LiveTestCase):
    def _provenance(self):
        install = ROOT / "install/p450-clean"
        renderer = (install / "share/bunker_sim_runtime/scripts/"
                    "render_bunker_runtime.py")
        source = (install / "share/bunker_description/urdf/"
                  "bunker.urdf.xacro")
        renderer_environment = dict(os.environ)
        renderer_environment["PYTHONPATH"] = (
            str(install / "lib/python3/dist-packages") + ":" +
            "/opt/ros/noetic/lib/python3/dist-packages")
        completed = subprocess.run(
            [str(renderer), str(source)], check=True,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            env=renderer_environment)
        install_model = self.run_dir / "install-bunker-runtime.urdf"
        install_model.write_bytes(completed.stdout)
        install_model.chmod(0o600)
        model = self.run_dir / "rendered-bunker.urdf"
        model.write_bytes(completed.stdout)
        model.chmod(0o600)
        package = install / "share/bunker_description"
        urdf_root = ET.fromstring(completed.stdout)
        meshes = []
        for element in urdf_root.findall(".//mesh"):
            uri = element.get("filename")
            relative = uri[len("package://bunker_description/"):]
            path = package / relative
            meshes.append({
                "uri": uri, "path": str(path),
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            })
        plugins = [{"basename": Path(path).name, "path": path}
                   for path in PLUGIN_PATHS]
        runtime_environment = LiveBoundaryTest._environment(self)
        install_environment = dict(runtime_environment["search_paths"])
        install_environment.update(runtime_environment["fixed_environment"])
        install_environment.update(runtime_environment["state_paths"])
        install_environment["GAZEBO_MODEL_DATABASE_URI"] = ""
        contract = {
            "schema_version": 1, "status": "PASS",
            "environment": install_environment,
            "packages": {
                "bunker_description": str(package),
                "bunker_sim_runtime": str(
                    install / "share/bunker_sim_runtime"),
            },
            "renderer": {
                "path": str(renderer),
                "sha256": hashlib.sha256(renderer.read_bytes()).hexdigest(),
                "input": str(source),
                "input_sha256": hashlib.sha256(
                    source.read_bytes()).hexdigest(),
                "output": str(install_model),
                "output_sha256": hashlib.sha256(
                    install_model.read_bytes()).hexdigest(),
            },
            "meshes": meshes,
            "plugins": [{"path": path, "ldd_status": "PASS"}
                        for path in PLUGIN_PATHS],
        }
        contract_path = self.run_dir / "install-contract.json"
        contract_path.write_text(
            json.dumps(contract, allow_nan=False, sort_keys=True,
                       separators=(",", ":")) + "\n", encoding="utf-8")
        contract_path.chmod(0o600)
        file_record = lambda path: {
            "path": str(path),
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        }
        return {
            "install_contract_path": str(contract_path),
            "install_contract_sha256": hashlib.sha256(
                contract_path.read_bytes()).hexdigest(),
            "packages": contract["packages"],
            "renderer": file_record(renderer),
            "input": file_record(source),
            "model": file_record(model),
            "meshes": meshes, "plugins": plugins,
            "external_before": {
                "p450_sim": {"head": "1" * 40, "status": ""},
                "px4": {"head": "2" * 40, "status": ""},
            },
        }

    def test_install_renderer_model_and_log_provenance(self):
        provenance = self._provenance()
        summary = live.validate_provenance(provenance)
        self.assertEqual(17, summary["mesh_count"])
        self.assertEqual(live.RUNTIME_URDF_SHA256,
                         summary["model_sha256"])
        changed = copy.deepcopy(provenance)
        changed["input"]["sha256"] = "0" * 64
        with self.assertRaises(live.LiveContractError):
            live.validate_provenance(changed)
        wrong_output = copy.deepcopy(provenance)
        contract_path = Path(wrong_output["install_contract_path"])
        contract = json.loads(contract_path.read_text(encoding="utf-8"))
        alias = self.run_dir / "unexpected-output.urdf"
        alias.write_bytes(Path(wrong_output["model"]["path"]).read_bytes())
        alias.chmod(0o600)
        contract["renderer"]["output"] = str(alias)
        contract_path.write_text(
            json.dumps(contract, allow_nan=False, sort_keys=True,
                       separators=(",", ":")) + "\n", encoding="utf-8")
        wrong_output["install_contract_sha256"] = hashlib.sha256(
            contract_path.read_bytes()).hexdigest()
        with self.assertRaises(live.LiveContractError):
            live.validate_provenance(wrong_output)

        wrong_environment = self._provenance()
        contract_path = Path(wrong_environment["install_contract_path"])
        contract = json.loads(contract_path.read_text(encoding="utf-8"))
        del contract["environment"]["PATH"]
        contract_path.write_text(
            json.dumps(contract, allow_nan=False, sort_keys=True,
                       separators=(",", ":")) + "\n", encoding="utf-8")
        wrong_environment["install_contract_sha256"] = hashlib.sha256(
            contract_path.read_bytes()).hexdigest()
        with self.assertRaises(live.LiveContractError):
            live.validate_provenance(wrong_environment)

        wrong_mesh = self._provenance()
        contract_path = Path(wrong_mesh["install_contract_path"])
        contract = json.loads(contract_path.read_text(encoding="utf-8"))
        unused_path = (ROOT / "install/p450-clean/share/"
                       "bunker_description/meshes/base_link.STL")
        replacement = {
            "uri": "package://bunker_description/meshes/base_link.STL",
            "path": str(unused_path),
            "sha256": hashlib.sha256(unused_path.read_bytes()).hexdigest(),
        }
        contract["meshes"][0] = replacement
        wrong_mesh["meshes"][0] = replacement
        contract_path.write_text(
            json.dumps(contract, allow_nan=False, sort_keys=True,
                       separators=(",", ":")) + "\n", encoding="utf-8")
        wrong_mesh["install_contract_sha256"] = hashlib.sha256(
            contract_path.read_bytes()).hexdigest()
        with self.assertRaises(live.LiveContractError):
            live.validate_provenance(wrong_mesh)

        launch_log = self.run_dir / "roslaunch.log"
        launch_log.write_text("clean fixture\n", encoding="utf-8")
        logs = {
            "launch_path": str(launch_log),
            "launch_sha256": hashlib.sha256(
                launch_log.read_bytes()).hexdigest(),
            "fatal_matches": [],
        }
        self.assertEqual(str(launch_log),
                         live.validate_logs(logs)["launch_path"])

    def _pass_documents(self):
        provenance = self._provenance()
        environment = LiveBoundaryTest._environment(self)
        expected = live.load_expected_fixed_transforms(
            provenance["model"]["path"], provenance["model"]["sha256"])
        raw = self.run_dir / "gz-model-info.txt"
        raw.write_text("fixture\n", encoding="utf-8")
        collision = {
            "model": "bunker", "link": "base_link",
            "collisions": [{
                "name": "base_link_collision", "link": "base_link",
                "geometry": "box",
                "origin": [0.018058912, 0.001357451, -0.160420741],
                "size": [1.026335219, 0.782744936, 0.395154782],
            }],
            "mesh_collision_count": 0, "baseline_height": 0.36,
            "final_height": 0.36, "roll": 0.0, "pitch": 0.0,
            "raw_info_path": str(raw),
            "raw_info_sha256": hashlib.sha256(raw.read_bytes()).hexdigest(),
        }
        seed = live.make_probe_failure_seed(
            "20260901T000000Z-123", self.run_dir,
            "2026-09-01T00:00:00Z", "probe not completed")
        probe = copy.deepcopy(seed)
        probe.update({
            "status": "PASS", "fatal_errors": [],
            "environment": environment, "provenance": provenance,
            "processes": {
                "owned_root": {
                    "pid": 123, "pgid": 123, "sid": 123,
                    "start_ticks": 1000,
                },
                "gzserver": {
                    "pid": 124, "pgid": 123, "sid": 123,
                    "start_ticks": 1001,
                    "executable": "/usr/bin/gzserver-11.15.1",
                    "plugin_maps": provenance["plugins"],
                },
                "models": ["bunker"],
                "required_nodes": ["/gazebo", "/ground/velocity_guard"],
            },
            "graph": {"snapshot_stamp": 50.0, "topics": _graph()["topics"]},
            "samples": {
                "clock": live.validate_clock_samples([49.8, 49.9, 50.0]),
                "odom": live.validate_odometry_samples(_odom_samples()),
                "scan": live.validate_scan_samples(_scan_samples()),
            },
            "guard": {
                "records": _guard_records(), "probe_disconnected": True,
                "post_probe_snapshot_stamp": 49.7,
            },
            "motion": {
                "forward": _motion("forward"),
                "rotation": _motion("rotation"),
            },
            "tf": {"edges": _tf_edges(expected), "forbidden_frames": []},
            "collision": collision,
        })
        probe["run"]["probe_state"] = "completed"
        launch_log = self.run_dir / "roslaunch.log"
        launch_log.write_text("clean fixture\n", encoding="utf-8")
        launcher = {
            "schema_version": 1,
            "run": {
                "run_id": probe["run"]["run_id"],
                "run_dir": probe["run"]["run_dir"],
                "started_utc": probe["run"]["started_utc"],
                "finished_utc": "2026-09-01T00:01:00Z",
                "wall_seconds": 60.0, "supervisor_status": 0,
            },
            "logs": {
                "launch_path": str(launch_log),
                "launch_sha256": hashlib.sha256(
                    launch_log.read_bytes()).hexdigest(),
                "fatal_matches": [],
            },
            "shutdown": {
                "probe_status": 0, "launcher_status": 0,
                "cleanup_owner": "inner", "signal_sequence": ["SIGINT"],
                "escalated": False, "remaining_pids": [],
                "remaining_ports": [],
            },
            "external_after": copy.deepcopy(provenance["external_before"]),
            "first_failure": None, "fatal_errors": [],
        }
        return probe, launcher

    def test_pass_document_runs_every_live_validator(self):
        probe, launcher = self._pass_documents()
        self.assertEqual("PASS", live.validate_probe_evidence(probe)["status"])
        result = live.merge_result(probe, launcher)
        self.assertEqual("PASS", result["status"])
        invalid = copy.deepcopy(probe)
        del invalid["graph"]["topics"]["/ground/scan"]
        with self.assertRaises(live.LiveContractError):
            live.validate_probe_evidence(invalid)

    def test_pass_sections_are_bound_to_declared_run(self):
        probe, _launcher = self._pass_documents()
        other_temporary = tempfile.TemporaryDirectory(dir=str(TEMP_ROOT))
        self.addCleanup(other_temporary.cleanup)
        other_run = Path(other_temporary.name)
        other_run.chmod(0o700)
        probe["run"]["run_dir"] = str(other_run)
        with self.assertRaises(live.LiveContractError):
            live.validate_probe_evidence(probe)

    def test_set_derived_lists_serialize_deterministically(self):
        first, _launcher = self._pass_documents()
        audit = first["environment"]["writable_fd_audit"]
        audit["checked_pids"] = [123, 124]
        audit["allowed_kernel_endpoints"] = ["pipe", "socket"]
        second = copy.deepcopy(first)
        second["tf"]["edges"].reverse()
        second["provenance"]["meshes"].reverse()
        second["provenance"]["plugins"].reverse()
        second["processes"]["gzserver"]["plugin_maps"].reverse()
        second["environment"]["writable_fd_audit"]["checked_pids"].reverse()
        second["environment"]["writable_fd_audit"][
            "allowed_kernel_endpoints"].reverse()
        first_normalized = live.validate_probe_evidence(first)
        second_normalized = live.validate_probe_evidence(second)
        serialize = lambda payload: json.dumps(
            payload, allow_nan=False, sort_keys=True, separators=(",", ":"))
        self.assertEqual(serialize(first_normalized),
                         serialize(second_normalized))

    def test_every_top_level_and_section_key_is_required(self):
        probe, launcher = self._pass_documents()
        result = live.merge_result(probe, launcher)
        for key in live.PROBE_KEYS:
            altered = copy.deepcopy(probe)
            del altered[key]
            with self.subTest(document="probe", key=key):
                with self.assertRaises(live.LiveContractError):
                    live.validate_probe_evidence(altered)
        for section, keys in live.PROBE_SECTION_KEYS.items():
            for key in keys:
                altered = copy.deepcopy(probe)
                del altered[section][key]
                with self.subTest(document="probe", section=section,
                                  key=key):
                    with self.assertRaises(live.LiveContractError):
                        live.validate_probe_evidence(altered)
        for key in live.LAUNCHER_KEYS:
            altered = copy.deepcopy(launcher)
            del altered[key]
            with self.subTest(document="launcher", key=key):
                with self.assertRaises(live.LiveContractError):
                    live.merge_result(probe, altered)
        for section, keys in (
                ("run", live.LAUNCHER_RUN_KEYS),
                ("logs", live.LOG_KEYS),
                ("shutdown", live.SHUTDOWN_KEYS)):
            for key in keys:
                altered = copy.deepcopy(launcher)
                del altered[section][key]
                with self.subTest(document="launcher", section=section,
                                  key=key):
                    with self.assertRaises(live.LiveContractError):
                        live.merge_result(probe, altered)
        for key in live.RESULT_KEYS:
            altered = copy.deepcopy(result)
            del altered[key]
            with self.subTest(document="result", key=key):
                with self.assertRaises(live.LiveContractError):
                    live.validate_live_evidence(altered)
        result_sections = dict(live.PROBE_SECTION_KEYS)
        result_sections.update({
            "run": live.RESULT_RUN_KEYS,
            "provenance": live.RESULT_PROVENANCE_KEYS,
            "logs": live.LOG_KEYS,
            "shutdown": live.SHUTDOWN_KEYS,
        })
        for section, keys in result_sections.items():
            for key in keys:
                altered = copy.deepcopy(result)
                del altered[section][key]
                with self.subTest(document="result", section=section,
                                  key=key):
                    with self.assertRaises(live.LiveContractError):
                        live.validate_live_evidence(altered)


class LiveDocumentTest(LiveTestCase):
    def _launcher(self, seed, failure=True):
        message = "startup failed" if failure else None
        return {
            "schema_version": 1,
            "run": {
                "run_id": seed["run"]["run_id"],
                "run_dir": seed["run"]["run_dir"],
                "started_utc": seed["run"]["started_utc"],
                "finished_utc": "2026-09-01T00:01:00Z",
                "wall_seconds": 60.0,
                "supervisor_status": 70 if failure else 0,
            },
            "logs": {
                "launch_path": None if failure else str(self.run_dir / "roslaunch.log"),
                "launch_sha256": None if failure else "0" * 64,
                "fatal_matches": [message] if failure else [],
            },
            "shutdown": {
                "probe_status": 70 if failure else 0,
                "launcher_status": 70 if failure else 0,
                "cleanup_owner": "inner", "signal_sequence": ["SIGINT"],
                "escalated": False, "remaining_pids": [],
                "remaining_ports": [],
            },
            "external_after": None,
            "first_failure": ({
                "exit_class": 70, "phase": "readiness", "message": message,
            } if failure else None),
            "fatal_errors": [message] if failure else [],
        }

    def test_probe_merge_result_and_atomic_write(self):
        seed = live.make_probe_failure_seed(
            "20260901T000000Z-123", self.run_dir,
            "2026-09-01T00:00:00Z", "probe not completed")
        validated = live.validate_probe_evidence(seed)
        self.assertEqual("seed", validated["run"]["probe_state"])
        launcher = self._launcher(seed)
        result = live.merge_result(seed, launcher)
        self.assertEqual(2, result["schema_version"])
        self.assertEqual("FAIL", result["status"])
        self.assertEqual("startup failed", result["fatal_errors"][0])
        self.assertNotIn("probe not completed", result["fatal_errors"])
        self.assertEqual("FAIL", live.validate_live_evidence(result)["status"])

        probe_path = self.run_dir / "probe-evidence.json"
        result_path = self.run_dir / "result.json"
        self.assertEqual(probe_path, live.atomic_write_probe(probe_path, seed))
        self.assertEqual(result_path, live.atomic_write_result(result_path, result))
        self.assertEqual(0o600, probe_path.stat().st_mode & 0o777)
        self.assertEqual(0o600, result_path.stat().st_mode & 0o777)
        self.assertEqual([], list(self.run_dir.glob(".*.tmp")))
        serialized = json.dumps(result, allow_nan=False, sort_keys=True,
                                separators=(",", ":"))
        self.assertEqual(serialized + "\n", result_path.read_text(encoding="utf-8"))

        for key in live.PROBE_KEYS:
            altered = copy.deepcopy(seed)
            del altered[key]
            with self.subTest(probe_key=key):
                with self.assertRaises(live.LiveContractError):
                    live.validate_probe_evidence(altered)
        for key in live.RESULT_KEYS:
            altered = copy.deepcopy(result)
            del altered[key]
            with self.subTest(result_key=key):
                with self.assertRaises(live.LiveContractError):
                    live.validate_live_evidence(altered)

    def test_atomic_writer_rejects_cross_run_destination(self):
        seed = live.make_probe_failure_seed(
            "20260901T000000Z-123", self.run_dir,
            "2026-09-01T00:00:00Z", "probe not completed")
        other_temporary = tempfile.TemporaryDirectory(dir=str(TEMP_ROOT))
        self.addCleanup(other_temporary.cleanup)
        other_run = Path(other_temporary.name)
        other_run.chmod(0o700)
        with self.assertRaises(live.LiveContractError):
            live.atomic_write_probe(
                other_run / "probe-evidence.json", seed)

    def test_cli_exit_classes_and_fixed_outputs(self):
        probe_path = self.run_dir / "probe-evidence.json"
        result_path = self.run_dir / "result.json"
        run_id = "20260901T000000Z-123"
        started = "2026-09-01T00:00:00Z"
        self.assertEqual(0, live.main([
            "seed-probe", "--run-id", run_id,
            "--run-dir", str(self.run_dir), "--started-utc", started,
            "--fatal-error", "probe not completed",
            "--output", str(probe_path),
        ]))
        self.assertEqual(1, live.main([
            "validate-probe", "--input", str(probe_path)]))
        seed = json.loads(probe_path.read_text(encoding="utf-8"))
        launcher_path = self.run_dir / "launcher-evidence.json"
        launcher_path.write_text(
            json.dumps(self._launcher(seed), allow_nan=False, sort_keys=True,
                       separators=(",", ":")) + "\n", encoding="utf-8")
        self.assertEqual(1, live.main([
            "finalize", "--probe", str(probe_path),
            "--launcher", str(launcher_path),
            "--output", str(result_path),
        ]))
        self.assertEqual(1, live.main([
            "validate-result", "--input", str(result_path)]))
        malformed = self.run_dir / "malformed.json"
        malformed.write_text('{"value":NaN}\n', encoding="utf-8")
        duplicate = self.run_dir / "duplicate.json"
        valid_result = result_path.read_text(encoding="utf-8")
        duplicate.write_text(
            '{"schema_version":999,' + valid_result[1:], encoding="utf-8")
        with contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(65, live.main([
                "validate-result", "--input", str(malformed)]))
            self.assertEqual(65, live.main([
                "validate-result", "--input", str(duplicate)]))
            self.assertEqual(64, live.main(["unknown-command"]))

    def test_completed_fail_rejects_malformed_available_section(self):
        seed = live.make_probe_failure_seed(
            "20260901T000000Z-123", self.run_dir,
            "2026-09-01T00:00:00Z", "probe not completed")
        seed["run"]["probe_state"] = "completed"
        seed["fatal_errors"] = ["graph collection failed"]
        seed["graph"] = {"snapshot_stamp": 1.0, "topics": {}}
        with self.assertRaises(live.LiveContractError):
            live.validate_probe_evidence(seed)

    def test_result_fail_rejects_malformed_available_section(self):
        seed = live.make_probe_failure_seed(
            "20260901T000000Z-123", self.run_dir,
            "2026-09-01T00:00:00Z", "probe not completed")
        result = live.merge_result(seed, self._launcher(seed))
        result["environment"] = {
            key: "garbage"
            for key in live.PROBE_SECTION_KEYS["environment"]
        }
        with self.assertRaises(live.LiveContractError):
            live.validate_live_evidence(result)

    def test_launcher_failure_requires_real_first_failure(self):
        seed = live.make_probe_failure_seed(
            "20260901T000000Z-123", self.run_dir,
            "2026-09-01T00:00:00Z", "probe not completed")
        launcher = self._launcher(seed)
        launcher["first_failure"] = None
        launcher["fatal_errors"] = []
        with self.assertRaises(live.LiveContractError):
            live.merge_result(seed, launcher)

    def test_shutdown_resource_sets_are_normalized(self):
        seed = live.make_probe_failure_seed(
            "20260901T000000Z-123", self.run_dir,
            "2026-09-01T00:00:00Z", "probe not completed")
        first_launcher = self._launcher(seed)
        first_launcher["shutdown"]["remaining_pids"] = [125, 124]
        first_launcher["shutdown"]["remaining_ports"] = [11372, 11371]
        second_launcher = copy.deepcopy(first_launcher)
        second_launcher["shutdown"]["remaining_pids"].reverse()
        second_launcher["shutdown"]["remaining_ports"].reverse()
        first = live.merge_result(seed, first_launcher)
        second = live.merge_result(seed, second_launcher)
        self.assertEqual(first, second)


class LiveMergeFailureTest(LiveTestCase):
    _provenance = LiveProvenanceTest._provenance
    _pass_documents = LiveProvenanceTest._pass_documents

    def test_outer_cleanup_is_materialized_as_schema_two_fail(self):
        probe, launcher = self._pass_documents()
        launcher["shutdown"]["cleanup_owner"] = "outer-watchdog"
        launcher["first_failure"] = {
            "exit_class": 70, "phase": "cleanup",
            "message": "cleanup owner is outer-watchdog",
        }
        launcher["fatal_errors"] = ["cleanup owner is outer-watchdog"]
        result = live.merge_result(probe, launcher)
        self.assertEqual("FAIL", result["status"])
        self.assertIn("outer-watchdog", result["fatal_errors"][0])


if __name__ == "__main__":
    unittest.main()
