#!/usr/bin/env python3
"""Focused tests for public map-goal tracking in the Prometheus facade."""

import importlib.util
import copy
from pathlib import Path
import sys
import types
import unittest
from unittest import mock

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
import rospy
import tf2_ros


PACKAGE = Path(__file__).resolve().parents[1]
SERVER = PACKAGE / "scripts/p450_flight_facade_node.py"
FACADE_LAUNCH = PACKAGE / "launch/p450_flight_facade.launch"
SIM_LAUNCH = PACKAGE.parent / "sim_platform_bringup/launch/air_ground_standalone.launch"
DEMO_LAUNCH = PACKAGE.parents[1] / "demos/air_ground_pick_demo/launch/air_ground_pick_demo.launch"


class _Feedback:
    def __init__(self):
        self.current_pose = PoseStamped()
        self.native_mode = ""
        self.position_error = 0.0


class _Result:
    def __init__(self, success=False, message=""):
        self.success = success
        self.message = message


def _load_server():
    prometheus_msg = types.ModuleType("prometheus_msgs.msg")
    for name in ("UAVCommand", "UAVControlState", "UAVSetup", "UAVState"):
        setattr(prometheus_msg, name, type(name, (), {}))
    prometheus = types.ModuleType("prometheus_msgs")
    prometheus.msg = prometheus_msg

    runtime_msg = types.ModuleType("robot_runtime_interfaces.msg")
    runtime_msg.FlightCommandAction = type("FlightCommandAction", (), {})
    runtime_msg.FlightCommandFeedback = _Feedback
    runtime_msg.FlightCommandResult = _Result
    runtime = types.ModuleType("robot_runtime_interfaces")
    runtime.msg = runtime_msg

    geometry_helpers = types.ModuleType("tf2_geometry_msgs")

    def do_transform_pose(target, transform):
        transformed = copy.deepcopy(target)
        transformed.header = transform.header
        transformed.pose.position.x += transform.transform.translation.x
        transformed.pose.position.y += transform.transform.translation.y
        transformed.pose.position.z += transform.transform.translation.z
        return transformed

    geometry_helpers.do_transform_pose = do_transform_pose

    source = str(PACKAGE / "src")
    if source not in sys.path:
        sys.path.insert(0, source)
    saved = {
        name: sys.modules.get(name)
        for name in ("prometheus_msgs", "prometheus_msgs.msg",
                     "robot_runtime_interfaces", "robot_runtime_interfaces.msg",
                     "tf2_geometry_msgs")
    }
    sys.modules.update({
        "prometheus_msgs": prometheus,
        "prometheus_msgs.msg": prometheus_msg,
        "robot_runtime_interfaces": runtime,
        "robot_runtime_interfaces.msg": runtime_msg,
        "tf2_geometry_msgs": geometry_helpers,
    })
    try:
        spec = importlib.util.spec_from_file_location("facade_node", str(SERVER))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        for name, previous in saved.items():
            if previous is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = previous


class _State:
    mode = "OFFBOARD"
    velocity = (0.0, 0.0, 0.0)


class _Server:
    def __init__(self, preempt=False):
        self.preempt = preempt
        self.succeeded = []
        self.preempted = []

    def is_preempt_requested(self):
        return self.preempt

    def set_succeeded(self, result):
        self.succeeded.append(result)

    def set_preempted(self, result):
        self.preempted.append(result)


class _TransformBuffer:
    def __init__(self, translations=None, failure=None):
        self.translations = translations or {}
        self.failure = failure
        self.calls = []
        self.after_lookup = lambda: None

    def lookup_transform(self, target_frame, source_frame, stamp, timeout):
        self.calls.append((target_frame, source_frame, stamp.to_sec()))
        self.after_lookup()
        if self.failure is not None:
            raise self.failure
        transform = TransformStamped()
        transform.header.frame_id = target_frame
        transform.child_frame_id = source_frame
        transform.header.stamp = stamp
        transform.transform.translation.x = self.translations.get(stamp.to_sec(), 0.0)
        transform.transform.rotation.w = 1.0
        return transform


def _pose(frame, stamp, x=10.0):
    target = PoseStamped()
    target.header.frame_id = frame
    target.header.stamp = rospy.Time.from_sec(stamp)
    target.pose.position.x = x
    target.pose.orientation.w = 1.0
    return target


def _snapshot(stamp, x):
    odom = Odometry()
    odom.header.frame_id = "uav1/odom"
    odom.header.stamp = rospy.Time.from_sec(stamp)
    odom.pose.pose.position.x = x
    return (_State(), object(), odom, 0.0, 0.0, 0.0)


def _facade(module, snapshots, transforms, preempt=False, healthy=True):
    facade = module.PrometheusFlightFacade.__new__(module.PrometheusFlightFacade)
    values = iter(snapshots)
    facade._snapshot = lambda: next(values)
    facade._health = lambda snapshot=None: healthy
    facade._tf_buffer = transforms
    facade._odom_frame = "uav1/odom"
    facade._position_tolerance = 0.15
    facade._fly_to_position_tolerance = 0.05
    facade._settle_speed = 0.05
    facade._publish_rate = 10.0
    facade._takeoff_height_param = "/test/takeoff_height"
    facade._server = _Server(preempt)
    facade._published = []
    facade._feedback_targets = []
    facade._aborts = []
    facade._publish_operation = lambda operation: facade._published.append(operation)
    facade._publish_command = lambda operation: facade._published.append(operation)
    facade._feedback = lambda target=None, snapshot=None: facade._feedback_targets.append(
        (target, snapshot))
    facade._abort = lambda message: facade._aborts.append(message)
    facade._succeed = lambda message: facade._server.succeeded.append(message)
    return facade


class MapGoalTrackingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = _load_server()
        rate_patch = mock.patch.object(
            cls.module.rospy, "Rate",
            side_effect=lambda _hz: types.SimpleNamespace(sleep=lambda: None))
        shutdown_patch = mock.patch.object(
            cls.module.rospy, "is_shutdown", return_value=False)
        rate_patch.start()
        shutdown_patch.start()
        cls.addClassCleanup(rate_patch.stop)
        cls.addClassCleanup(shutdown_patch.stop)

    def test_map_goal_uses_each_fresh_odom_stamp_for_command_completion_feedback(self):
        transforms = _TransformBuffer({11.0: -1.0, 12.0: -2.0})
        snapshots = [_snapshot(11.0, 8.9), _snapshot(12.0, 8.0)]
        facade = _facade(self.module, snapshots, transforms)
        facade._execute_fly_to(types.SimpleNamespace(target=_pose("/map", 7.0)))

        self.assertEqual([11.0, 12.0], [call[2] for call in transforms.calls])
        self.assertEqual((9.0, 0.0, 0.0, 0.0), facade._published[0][2])
        self.assertEqual((8.0, 0.0, 0.0, 0.0), facade._published[-1][2])
        self.assertEqual((9.0, 0.0, 0.0, 0.0), facade._feedback_targets[0][0])
        self.assertIs(snapshots[0], facade._feedback_targets[0][1])
        self.assertEqual(["target reached"], facade._server.succeeded)

        initial_transforms = _TransformBuffer({21.0: -2.0})
        initial = _facade(
            self.module, [_snapshot(21.0, 8.0)], initial_transforms)
        initial._execute_fly_to(
            types.SimpleNamespace(target=_pose("map", 7.0)))
        self.assertEqual(
            [("command", "Move_XYZ_POS", (8.0, 0.0, 0.0, 0.0))],
            initial._published)
        self.assertEqual(["target reached"], initial._server.succeeded)

    def test_takeoff_keeps_base_tolerance_when_fly_to_is_tighter(self):
        facade = _facade(
            self.module, [_snapshot(1.0, 0.0), _snapshot(2.0, 0.0)],
            _TransformBuffer())
        facade._wait_native = lambda *args, **kwargs: True
        completion = mock.Mock(return_value=True)
        with mock.patch.object(self.module.rospy, "get_param", return_value=1.0), \
                mock.patch.object(self.module, "position_complete", completion):
            facade._execute_takeoff()
        self.assertEqual(0.15, completion.call_args.args[3])
        self.assertEqual(["takeoff complete"], facade._server.succeeded)

    def test_stamp_override_does_not_mutate_the_public_target(self):
        target = _pose("/map", 7.0, x=3.0)
        transforms = _TransformBuffer({11.0: 2.0})
        facade = _facade(self.module, [], transforms)
        transformed = facade._target_in_odom(target, rospy.Time.from_sec(11.0))
        self.assertEqual((5.0, 0.0, 0.0, 0.0), transformed)
        facade._target_in_odom(target)
        self.assertEqual(7.0, target.header.stamp.to_sec())
        self.assertEqual("/map", target.header.frame_id)
        self.assertEqual(3.0, target.pose.position.x)

    def test_non_map_goal_keeps_one_shot_request_stamp_semantics(self):
        transforms = _TransformBuffer({7.0: -1.0})
        snapshots = [_snapshot(11.0, 8.9), _snapshot(12.0, 9.0)]
        facade = _facade(self.module, snapshots, transforms)
        facade._execute_fly_to(types.SimpleNamespace(target=_pose("mission", 7.0)))
        self.assertEqual([7.0], [call[2] for call in transforms.calls])
        self.assertEqual((9.0, 0.0, 0.0, 0.0), facade._published[0][2])
        self.assertEqual(["target reached"], facade._server.succeeded)

    def test_map_goal_preserves_preemption_health_and_tf_failure_paths(self):
        preempted = _facade(self.module, [], _TransformBuffer(), preempt=True)
        preempted._execute_fly_to(types.SimpleNamespace(target=_pose("map", 7.0)))
        self.assertEqual(["Current_Pos_Hover"], preempted._published)
        self.assertEqual(1, len(preempted._server.preempted))

        during_lookup_buffer = _TransformBuffer({11.0: -2.0})
        during_lookup = _facade(
            self.module, [_snapshot(11.0, 8.0)], during_lookup_buffer)
        during_lookup_buffer.after_lookup = lambda: setattr(
            during_lookup._server, "preempt", True)
        during_lookup._execute_fly_to(
            types.SimpleNamespace(target=_pose("map", 7.0)))
        self.assertEqual(["Current_Pos_Hover"], during_lookup._published)
        self.assertEqual(1, len(during_lookup._server.preempted))
        self.assertEqual([], during_lookup._server.succeeded)

        unhealthy = _facade(
            self.module, [_snapshot(11.0, 0.0)], _TransformBuffer(), healthy=False)
        unhealthy._execute_fly_to(types.SimpleNamespace(target=_pose("map", 7.0)))
        self.assertEqual(
            ["Prometheus backend unavailable, stale, or failed"], unhealthy._aborts)

        failed_tf = _facade(
            self.module, [_snapshot(11.0, 0.0)],
            _TransformBuffer(failure=tf2_ros.LookupException("missing")))
        failed_tf._execute(types.SimpleNamespace(
            command=self.module.FLY_TO, target=_pose("map", 7.0)))
        self.assertEqual(
            ["FLY_TO target cannot be transformed"], failed_tf._aborts)


class PositionToleranceLaunchTest(unittest.TestCase):
    def test_default_and_forwarding_exist_through_the_demo_chain(self):
        server = SERVER.read_text(encoding="utf-8")
        facade = FACADE_LAUNCH.read_text(encoding="utf-8")
        sim = SIM_LAUNCH.read_text(encoding="utf-8")
        demo = DEMO_LAUNCH.read_text(encoding="utf-8")
        self.assertIn(
            '"fly_to_position_tolerance", self._position_tolerance)', server)
        self.assertIn('<arg name="position_tolerance" default="0.15"/>', facade)
        self.assertIn('<param name="position_tolerance" value="$(arg position_tolerance)"/>', facade)
        self.assertIn('<arg name="fly_to_position_tolerance" default="$(arg position_tolerance)"/>', facade)
        self.assertIn('<param name="fly_to_position_tolerance" value="$(arg fly_to_position_tolerance)"/>', facade)
        self.assertIn('<arg name="flight_position_tolerance" default="0.15"/>', sim)
        self.assertIn('<arg name="fly_to_position_tolerance" value="$(arg flight_position_tolerance)"/>', sim)
        self.assertIn('<arg name="flight_position_tolerance" default="0.15"/>', demo)
        self.assertIn('<arg name="flight_position_tolerance" value="$(arg flight_position_tolerance)"/>', demo)


if __name__ == "__main__":
    unittest.main()
