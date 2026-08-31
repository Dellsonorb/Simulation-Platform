#!/usr/bin/env python3

import math
import pathlib
import unittest

import yaml

from brick_aerial_perception.viewpoint_mission import MissionState, ViewpointMission


PACKAGE = pathlib.Path(__file__).resolve().parents[1]
NODE = PACKAGE / "scripts" / "aerial_viewpoint_mission.py"


class ViewpointConfigurationTest(unittest.TestCase):
    def test_configuration_has_three_to_five_named_fixed_viewpoints(self):
        config = yaml.safe_load((PACKAGE / "config" / "viewpoints.yaml").read_text())
        viewpoints = config["viewpoints"]
        self.assertGreaterEqual(len(viewpoints), 3)
        self.assertLessEqual(len(viewpoints), 5)
        self.assertEqual(len({item["id"] for item in viewpoints}), len(viewpoints))
        for item in viewpoints:
            self.assertEqual(len(item["position"]), 3)
            self.assertIn("yaw", item)


class ViewpointMissionTest(unittest.TestCase):
    def setUp(self):
        self.mission = ViewpointMission(
            [
                {"id": "center", "position": [-1.6, 0.0, 1.4], "yaw": 0.0},
                {"id": "left", "position": [-1.6, -0.6, 1.4], "yaw": 0.35},
            ],
            arrival_tolerance=0.20,
            yaw_tolerance=0.12,
            settle_time=1.0,
            waypoint_timeout=10.0,
            max_settle_linear_speed=0.10,
            max_settle_angular_speed=0.10,
        )

    def test_requires_arrival_and_continuous_stability_before_sampling(self):
        self.mission.start(0.0)
        self.assertEqual(self.mission.state, MissionState.MOVING)
        self.mission.observe([-1.59, 0.01, 1.41], 0.03, 0.20, 0.0, 1.0)
        self.assertEqual(self.mission.state, MissionState.MOVING)
        self.mission.observe([-1.59, 0.01, 1.41], 0.03, 0.02, 0.01, 2.0)
        self.mission.observe([-1.60, 0.00, 1.40], 0.02, 0.01, 0.01, 2.8)
        self.assertEqual(self.mission.state, MissionState.MOVING)
        event = self.mission.observe([-1.60, 0.00, 1.40], 0.02, 0.01, 0.01, 3.1)
        self.assertEqual(event, "sampling_started")
        self.assertEqual(self.mission.state, MissionState.SAMPLING)

    def test_unstable_sample_resets_settle_latch(self):
        self.mission.start(0.0)
        self.mission.observe([-1.6, 0.0, 1.4], 0.0, 0.0, 0.0, 1.0)
        self.mission.observe([-1.2, 0.0, 1.4], 0.0, 0.0, 0.0, 1.5)
        self.mission.observe([-1.6, 0.0, 1.4], 0.0, 0.0, 0.0, 2.0)
        self.mission.observe([-1.6, 0.0, 1.4], 0.0, 0.0, 0.0, 2.6)
        self.assertEqual(self.mission.state, MissionState.MOVING)

    def test_sample_completion_advances_to_next_viewpoint(self):
        self.mission.start(0.0)
        self.mission.observe([-1.6, 0.0, 1.4], 0.0, 0.0, 0.0, 1.0)
        self.mission.observe([-1.6, 0.0, 1.4], 0.0, 0.0, 0.0, 2.1)
        event = self.mission.finish_sample(3.0)
        self.assertEqual(event, "next_viewpoint")
        self.assertEqual(self.mission.current["id"], "left")
        self.assertEqual(self.mission.state, MissionState.MOVING)
        self.assertEqual(["center"], self.mission.completed_ids)

    def test_final_sample_can_hold_ready_without_completing_or_landing(self):
        self.mission.start(0.0)
        self.mission.state = MissionState.SAMPLING
        self.mission.index = 1

        event = self.mission.finish_sample(3.0, hold_on_final=True)

        self.assertEqual("sequence_ready", event)
        self.assertEqual(MissionState.SAMPLING, self.mission.state)
        self.assertEqual(["left"], self.mission.completed_ids)
        repeated = self.mission.finish_sample(3.1, hold_on_final=True)
        self.assertEqual("sequence_ready", repeated)
        self.assertEqual(["left"], self.mission.completed_ids)

    def test_default_final_sample_still_completes_existing_mission(self):
        self.mission.start(0.0)
        self.mission.state = MissionState.SAMPLING
        self.mission.index = 1

        event = self.mission.finish_sample(3.0)

        self.assertEqual("mission_complete", event)
        self.assertEqual(MissionState.COMPLETE, self.mission.state)
        self.assertEqual(["left"], self.mission.completed_ids)

    def test_start_clears_completed_viewpoint_audit(self):
        self.mission.start(0.0)
        self.mission.state = MissionState.SAMPLING
        self.mission.finish_sample(1.0)
        self.assertEqual(["center"], self.mission.completed_ids)

        self.mission.start(2.0)

        self.assertEqual([], self.mission.completed_ids)

    def test_timeout_enters_failed_terminal_state(self):
        self.mission.start(0.0)
        event = self.mission.observe([0.0, 0.0, 1.4], math.pi, 0.0, 0.0, 10.1)
        self.assertEqual(event, "waypoint_timeout")
        self.assertEqual(self.mission.state, MissionState.FAILED)


class ViewpointMissionNodeContractTest(unittest.TestCase):
    def test_start_service_waits_for_positive_ros_time_before_epoch_capture(self):
        source = NODE.read_text(encoding="utf-8")
        self.assertIn('rospy.get_param("~clock_start_timeout", 5.0)', source)
        self.assertIn("def _wait_for_positive_ros_time", source)
        callback = source.split("def _start_cb", 1)[1].split(
            "\n    def ", 1)[0]
        self.assertLess(callback.index("_wait_for_positive_ros_time"),
                        callback.index("sequence_generation += 1"))
        self.assertIn("ROS_CLOCK_NOT_READY", callback)

    def test_final_hold_is_opt_in_and_sequence_status_is_latched(self):
        source = NODE.read_text(encoding="utf-8")
        for token in (
            'rospy.get_param("~hold_after_last_sample", False)',
            '"/m1/viewpoint_sequence_status"',
            "latch=True",
            "completed_viewpoints",
            "viewpoint_ids",
            "active_viewpoint",
            "sequence_generation",
            "sequence_ready",
            "hold_on_final=self.hold_after_last_sample",
        ):
            self.assertIn(token, source)

    def test_ready_sequence_hovers_until_existing_abort_or_landing_path(self):
        source = NODE.read_text(encoding="utf-8")
        sampling = source.split(
            'if self.flight_state.startswith("SAMPLING:")', 1
        )[1].split('if self.flight_state == "LANDING"', 1)[0]
        self.assertIn("Current_Pos_Hover", sampling)
        self.assertIn('event == "sequence_ready"', sampling)
        self.assertIn("self.sequence_ready = True", sampling)
        ready_branch = sampling.split('event == "sequence_ready"', 1)[1].split(
            "return", 1
        )[0]
        self.assertNotIn("UAVCommand.Land", ready_branch)
        self.assertIn("UAVCommand.Land", source)
        self.assertIn("operator_abort", source)


if __name__ == "__main__":
    unittest.main()
