"""Pure state and gating logic for predefined aerial viewpoints."""

import math
from enum import Enum

import numpy as np


class MissionState(Enum):
    IDLE = "IDLE"
    MOVING = "MOVING"
    SAMPLING = "SAMPLING"
    COMPLETE = "COMPLETE"
    FAILED = "FAILED"


def angle_error(first, second):
    return abs((float(first) - float(second) + math.pi) % (2.0 * math.pi) - math.pi)


class ViewpointMission:
    def __init__(self, viewpoints, arrival_tolerance, yaw_tolerance, settle_time,
                 waypoint_timeout, max_settle_linear_speed, max_settle_angular_speed):
        if not viewpoints:
            raise ValueError("at least one viewpoint is required")
        self.viewpoints = list(viewpoints)
        self.arrival_tolerance = float(arrival_tolerance)
        self.yaw_tolerance = float(yaw_tolerance)
        self.settle_time = float(settle_time)
        self.waypoint_timeout = float(waypoint_timeout)
        self.max_settle_linear_speed = float(max_settle_linear_speed)
        self.max_settle_angular_speed = float(max_settle_angular_speed)
        self.state = MissionState.IDLE
        self.index = 0
        self.phase_started = None
        self.settle_started = None
        self.completed_ids = []

    @property
    def current(self):
        return self.viewpoints[self.index]

    def start(self, now):
        self.index = 0
        self.state = MissionState.MOVING
        self.phase_started = float(now)
        self.settle_started = None
        self.completed_ids = []

    def observe(self, position, yaw, linear_speed, angular_speed, now):
        if self.state != MissionState.MOVING:
            return None
        now = float(now)
        if now - self.phase_started > self.waypoint_timeout:
            self.state = MissionState.FAILED
            self.settle_started = None
            return "waypoint_timeout"
        target = np.asarray(self.current["position"], dtype=np.float64)
        position_error = float(np.linalg.norm(np.asarray(position, dtype=np.float64) - target))
        stable = bool(
            position_error <= self.arrival_tolerance
            and angle_error(yaw, self.current["yaw"]) <= self.yaw_tolerance
            and float(linear_speed) <= self.max_settle_linear_speed
            and float(angular_speed) <= self.max_settle_angular_speed
        )
        if not stable:
            self.settle_started = None
            return None
        if self.settle_started is None:
            self.settle_started = now
            return None
        if now - self.settle_started < self.settle_time:
            return None
        self.state = MissionState.SAMPLING
        return "sampling_started"

    def finish_sample(self, now, hold_on_final=False):
        if self.state != MissionState.SAMPLING:
            return None
        viewpoint_id = self.current["id"]
        if viewpoint_id not in self.completed_ids:
            self.completed_ids.append(viewpoint_id)
        if self.index + 1 >= len(self.viewpoints):
            if hold_on_final:
                return "sequence_ready"
            self.state = MissionState.COMPLETE
            return "mission_complete"
        self.index += 1
        self.state = MissionState.MOVING
        self.phase_started = float(now)
        self.settle_started = None
        return "next_viewpoint"
