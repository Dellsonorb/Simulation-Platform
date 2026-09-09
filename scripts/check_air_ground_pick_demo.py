#!/usr/bin/env python3
"""Bounded end-to-end check for the natural Air-Ground Pick Demo."""

import argparse
from collections import deque
import json
import math
from pathlib import Path
import threading
import time


class DemoCheckError(RuntimeError):
    pass


EXPECTED_STATES = (
    "PREFLIGHT",
    "ARMING",
    "COMMAND_CONTROL",
    "TAKEOFF",
    "AIR_VIEW",
    "AIR_OBSERVE",
    "AIR_HANDOFF",
    "LANDING",
    "GROUND_APPROACH",
    "GROUND_STOPPED",
    "GROUND_OBSERVE",
    "GROUND_REFINED",
    "PREGRASP",
    "GRASP",
    "LIFTING",
    "LIFT",
)


def _finite(value, label):
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise DemoCheckError("%s is not finite" % label) from error
    if not math.isfinite(result):
        raise DemoCheckError("%s is not finite" % label)
    return result


def _event(events, state):
    return next(event for event in events if event.get('state') == state)


def status_sequence_summary(
        events, maximum_ground_travel, minimum_tcp_lift, ground_only=False):
    """Validate the one-shot demo states and their physical bounds."""
    states = [event.get("state") for event in events]
    expected = EXPECTED_STATES[8:] if ground_only else EXPECTED_STATES
    if states != list(expected):
        raise DemoCheckError(
            "status sequence is %s, expected %s" %
            (states, list(expected)))
    status_times = [
        _finite(event.get("ros_time"), "%s ros_time" % event["state"])
        for event in events
    ]
    if any(second < first for first, second in zip(
            status_times, status_times[1:])):
        raise DemoCheckError("status timestamps moved backwards")

    stopped = _event(events, "GROUND_STOPPED")
    ground_travel = _finite(
        stopped.get("ground_travel"), "ground_travel")
    travel_bound = _finite(maximum_ground_travel, "maximum_ground_travel")
    if ground_travel < 0.0 or ground_travel > travel_bound:
        raise DemoCheckError(
            "Ground travel %.3f m exceeds %.3f m" %
            (ground_travel, travel_bound))

    lifted = _event(events, "LIFT")
    tcp_lift = _finite(lifted.get("tcp_lift"), "tcp_lift")
    minimum = _finite(minimum_tcp_lift, "minimum_tcp_lift")
    if tcp_lift < minimum:
        raise DemoCheckError(
            "AUBO TCP lift %.3f m is below %.3f m" % (tcp_lift, minimum))
    if lifted.get("grasp_confirmed") is not True:
        raise DemoCheckError("LIFT status did not retain grasp confirmation")

    return {
        "states": states,
        "takeoff_count": states.count("TAKEOFF"),
        "landing_count": states.count("LANDING"),
        "ground_travel_m": ground_travel,
        "tcp_lift_m": tcp_lift,
    }


def observation_summary(
        samples, status_time, expected_stamp, expected_frame,
        maximum_age, stamp_tolerance=0.05):
    """Match the status handoff to a fresh pose actually seen on its topic."""
    status_time = _finite(status_time, "observation status time")
    expected_stamp = _finite(expected_stamp, "observation stamp")
    maximum_age = _finite(maximum_age, "maximum observation age")
    age = status_time - expected_stamp
    if age < -0.1 or age > maximum_age:
        raise DemoCheckError(
            "observation age %.3f s is outside [-0.1, %.3f]" %
            (age, maximum_age))

    matches = []
    for sample in samples:
        stamp = _finite(sample.get("stamp"), "topic observation stamp")
        if (sample.get("frame", "").lstrip("/") ==
                expected_frame.lstrip("/") and
                abs(stamp - expected_stamp) <= stamp_tolerance):
            matches.append((abs(stamp - expected_stamp), stamp))
    if not matches:
        raise DemoCheckError(
            "no %s observation matched status stamp %.6f" %
            (expected_frame, expected_stamp))
    matched_stamp = min(matches)[1]
    return {
        "frame": expected_frame.lstrip("/"),
        "stamp": matched_stamp,
        "age_s": age,
    }


def flight_cycle_summary(armed_samples):
    """Require one armed interval followed by a disarmed landing."""
    samples = tuple(bool(value) for value in armed_samples)
    armed_segments = sum(
        value and (index == 0 or not samples[index - 1])
        for index, value in enumerate(samples))
    if armed_segments != 1:
        raise DemoCheckError(
            "expected one armed interval, observed %d" % armed_segments)
    first_armed = samples.index(True)
    landed = any(not value for value in samples[first_armed + 1:])
    if not landed:
        raise DemoCheckError("P450 did not disarm after its flight")
    return {"armed_seen": True, "landed_after_arm": True}


def controller_success_summary(
        successful_goals, after_stamp, minimum_successes):
    """Count unique successful AUBO goals started after near-field sensing."""
    after_stamp = _finite(after_stamp, "controller cutoff stamp")
    unique = {
        str(goal_id): stamp
        for stamp, goal_id in successful_goals
        if _finite(stamp, "controller success stamp") >= after_stamp
    }
    if len(unique) < int(minimum_successes):
        raise DemoCheckError(
            "AUBO completed %d post-observation goals, expected at least %d" %
            (len(unique), int(minimum_successes)))
    return {
        "success_count": len(unique),
        "goal_ids": sorted(unique),
    }


def grasp_confirmation_summary(observed):
    if observed is not True:
        raise DemoCheckError("backend never confirmed the AG95 grasp")
    return {"grasp_confirmed": True}


def target_lift_summary(baseline_z, final_z, minimum_lift):
    """Use Gazebo pose only as an external E2E test oracle."""
    baseline = _finite(baseline_z, "target baseline z")
    final = _finite(final_z, "target final z")
    minimum = _finite(minimum_lift, "minimum target lift")
    lift = final - baseline
    if lift < minimum:
        raise DemoCheckError(
            "physical target lift %.3f m is below %.3f m" %
            (lift, minimum))
    return {
        "baseline_z_m": baseline,
        "final_z_m": final,
        "target_lift_m": lift,
    }


def finalized_summary(payload):
    """Mark checks final only after the wrapper proves clean teardown."""
    if not isinstance(payload, dict) or payload.get("status") != "CHECKS_PASS":
        raise DemoCheckError("summary is not awaiting wrapper finalization")
    result = dict(payload)
    result["status"] = "PASS"
    return result


class DemoMonitor:
    def __init__(self, rospy, goal_succeeded, target_model, ground_only=False):
        self._expected_states = EXPECTED_STATES[8:] if ground_only else EXPECTED_STATES
        self._rospy = rospy
        self._goal_succeeded = goal_succeeded
        self._target_model = target_model
        self._lock = threading.Lock()
        self.events = []
        self.air_observations = deque(maxlen=5000)
        self.ground_observations = deque(maxlen=5000)
        # Retain state transitions, not every 50 Hz sample. RM4D planning can
        # make the post-landing ground phase longer than the old sample window
        # and must not erase the already observed armed interval.
        self.armed_samples = deque(maxlen=32)
        self.arm_successes = {}
        self.grasp_confirmed = False
        self.target_baseline_z = None
        self.target_latest_z = None
        self.error = None
        self.terminal = threading.Event()

    def status(self, message):
        try:
            payload = json.loads(message.data)
            state = payload.get("state")
            if state == "FAILED":
                raise DemoCheckError(
                    "demo reported FAILED: %s" % payload.get("reason", ""))
            with self._lock:
                if state in self._expected_states:
                    if self.events and self.events[-1].get("state") == state:
                        return
                    expected_index = len(self.events)
                    if (expected_index >= len(self._expected_states) or
                            state != self._expected_states[expected_index]):
                        raise DemoCheckError(
                            "unexpected status %s after %s" %
                            (state, [item.get("state")
                                     for item in self.events]))
                    self.events.append(payload)
                    if state == "LIFT":
                        self.terminal.set()
        except (DemoCheckError, TypeError, ValueError) as error:
            with self._lock:
                self.error = str(error)
            self.terminal.set()

    @staticmethod
    def _pose_sample(message):
        return {
            "stamp": message.header.stamp.to_sec(),
            "frame": message.header.frame_id,
        }

    def air_pose(self, message):
        with self._lock:
            self.air_observations.append(self._pose_sample(message))

    def ground_pose(self, message):
        with self._lock:
            self.ground_observations.append(self._pose_sample(message))

    def uav_state(self, message):
        with self._lock:
            armed = bool(message.armed)
            if not self.armed_samples or self.armed_samples[-1] != armed:
                self.armed_samples.append(armed)

    def arm_status(self, message):
        receipt_stamp = self._rospy.Time.now().to_sec()
        with self._lock:
            for status in message.status_list:
                if status.status == self._goal_succeeded:
                    goal_id = status.goal_id.id or (
                        "stamp-%.9f" % status.goal_id.stamp.to_sec())
                    self.arm_successes.setdefault(goal_id, receipt_stamp)

    def grasp(self, message):
        if bool(message.data):
            with self._lock:
                self.grasp_confirmed = True

    def models(self, message):
        try:
            index = message.name.index(self._target_model)
            target_z = float(message.pose[index].position.z)
        except (ValueError, IndexError, TypeError):
            return
        if not math.isfinite(target_z):
            return
        with self._lock:
            self.target_latest_z = target_z
            states = [event.get("state") for event in self.events]
            before_grasp = "GRASP" not in states
            if before_grasp:
                if self.target_baseline_z is None:
                    self.target_baseline_z = target_z
                else:
                    self.target_baseline_z = min(
                        self.target_baseline_z, target_z)

    def snapshot(self):
        with self._lock:
            return {
                "events": [dict(event) for event in self.events],
                "air_observations": tuple(self.air_observations),
                "ground_observations": tuple(self.ground_observations),
                "armed_samples": tuple(self.armed_samples),
                "arm_successes": tuple(
                    (stamp, goal_id)
                    for goal_id, stamp in self.arm_successes.items()),
                "grasp_confirmed": self.grasp_confirmed,
                "target_baseline_z": self.target_baseline_z,
                "target_latest_z": self.target_latest_z,
                "error": self.error,
            }


def _write_summary(path, payload):
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8")


def run(args):
    import rospy
    from actionlib_msgs.msg import GoalStatus, GoalStatusArray
    from gazebo_msgs.msg import ModelStates
    from geometry_msgs.msg import PoseStamped
    from prometheus_msgs.msg import UAVState
    from std_msgs.msg import Bool, String

    rospy.init_node("check_air_ground_pick_demo", anonymous=True)
    monitor = DemoMonitor(
        rospy, GoalStatus.SUCCEEDED, args.target_model, ground_only=args.ground_only)
    subscriptions = (
        rospy.Subscriber(args.status_topic, String, monitor.status,
                         queue_size=50),
        rospy.Subscriber(args.air_pose_topic, PoseStamped, monitor.air_pose,
                         queue_size=20),
        rospy.Subscriber(
            args.ground_pose_topic, PoseStamped, monitor.ground_pose,
            queue_size=20),
        rospy.Subscriber(args.uav_state_topic, UAVState, monitor.uav_state,
                         queue_size=20),
        rospy.Subscriber(
            args.arm_status_topic, GoalStatusArray, monitor.arm_status,
            queue_size=20),
        rospy.Subscriber(
            args.grasp_confirmed_topic, Bool, monitor.grasp,
            queue_size=20),
        rospy.Subscriber(
            args.model_states_topic, ModelStates, monitor.models,
            queue_size=20),
    )
    monitor.subscriptions = subscriptions

    deadline = time.monotonic() + args.timeout
    while (not rospy.is_shutdown() and not monitor.terminal.is_set() and
           time.monotonic() < deadline):
        time.sleep(0.05)
    if not monitor.terminal.is_set():
        raise DemoCheckError("timed out waiting for LIFT")
    time.sleep(0.20)
    captured = monitor.snapshot()
    if captured["error"]:
        raise DemoCheckError(captured["error"])

    events = captured["events"]
    sequence = status_sequence_summary(
        events, args.maximum_ground_travel, args.minimum_lift, ground_only=args.ground_only)
    ground_event = _event(events, "GROUND_REFINED")
    aerial = flight = None
    if not args.ground_only:
        air_event = _event(events, "AIR_HANDOFF")
        aerial = observation_summary(
            captured["air_observations"], air_event["ros_time"],
            air_event.get("observation_stamp"), args.map_frame,
            args.maximum_observation_age)
        flight = flight_cycle_summary(captured["armed_samples"])
    ground = observation_summary(
        captured["ground_observations"], ground_event["ros_time"],
        ground_event.get("observation_stamp"), args.map_frame,
        args.maximum_observation_age)
    controllers = controller_success_summary(
        captured["arm_successes"], ground_event["ros_time"],
        args.minimum_arm_successes)
    grasp = grasp_confirmation_summary(captured["grasp_confirmed"])
    physical_lift = target_lift_summary(
        captured["target_baseline_z"], captured["target_latest_z"],
        args.minimum_lift)
    return {
        "status": "CHECKS_PASS",
        "kind": "GROUND_SEGMENT_CHECK" if args.ground_only else "AIR_GROUND_E2E_CHECK",
        "sequence": sequence,
        "observations": {"aerial": aerial, "ground": ground},
        "flight": flight,
        "arm_controller": controllers,
        "grasp": grasp,
        "physical_target": physical_lift,
    }


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Check one natural Air-Ground Pick Demo run")
    output = parser.add_mutually_exclusive_group(required=True)
    output.add_argument("--summary")
    output.add_argument("--finalize-summary")
    parser.add_argument("--timeout", type=float, default=300.0)
    parser.add_argument('--ground-only', action='store_true',
                        help='Check real Ground stages only, conditioned on an archived aerial handoff')
    parser.add_argument(
        "--status-topic", default="/air_ground_pick_demo/status")
    parser.add_argument("--air-pose-topic", default="/air_observer/target_pose")
    parser.add_argument(
        "--ground-pose-topic", default="/ground_observer/target_pose")
    parser.add_argument("--uav-state-topic", default="/uav1/prometheus/state")
    parser.add_argument(
        "--arm-status-topic",
        default="/ground/arm_controller/follow_joint_trajectory/status")
    parser.add_argument(
        "--grasp-confirmed-topic",
        default="/ground/gripper/grasp_confirmed")
    parser.add_argument("--model-states-topic", default="/gazebo/model_states")
    parser.add_argument("--target-model", default="pick_target")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--maximum-observation-age", type=float, default=1.0)
    parser.add_argument("--maximum-ground-travel", type=float, default=1.10)
    parser.add_argument("--minimum-lift", type=float, default=0.10)
    parser.add_argument("--minimum-arm-successes", type=int, default=3)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if args.finalize_summary:
        try:
            path = Path(args.finalize_summary)
            payload = json.loads(path.read_text(encoding="utf-8"))
            _write_summary(path, finalized_summary(payload))
        except Exception as error:
            print("FAIL: %s" % error)
            return 1
        print("PASS: finalized natural Air-Ground Pick Demo summary")
        return 0
    payload = {"status": "FAIL"}
    try:
        payload = run(args)
    except Exception as error:
        payload["error"] = str(error)
        _write_summary(args.summary, payload)
        print("FAIL: %s" % error)
        return 1
    _write_summary(args.summary, payload)
    print("PASS: natural Air-Ground Pick Demo checks")
    print("Summary: %s" % args.summary)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
