"""Pure initial-heading gate; no ROS imports."""
from __future__ import division

import math


def normalize_angle(value):
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi


class GoalLifecycle(object):
    """Track one action goal and ignore stale cancel/result events."""

    def __init__(self):
        self.clear()

    def start(self, goal_id, goal_stamp):
        self.active_goal_id = str(goal_id)
        self.active_goal_stamp = float(goal_stamp)

    def clear(self):
        self.active_goal_id = None
        self.active_goal_stamp = None

    def cancel(self, goal_id, cancel_stamp):
        if self.active_goal_id is None:
            return False
        goal_id = str(goal_id)
        cancel_stamp = float(cancel_stamp)
        matches = (goal_id == self.active_goal_id if goal_id else
                   (cancel_stamp == 0.0 or
                    self.active_goal_stamp <= cancel_stamp))
        if matches:
            self.clear()
        return matches

    def finish(self, goal_id):
        if (self.active_goal_id is None or
                str(goal_id) != self.active_goal_id):
            return False
        self.clear()
        return True


class HeadingGate(object):
    IDLE = 'IDLE'
    ALIGNING = 'ALIGNING'
    PASS_THROUGH = 'PASS_THROUGH'
    FAILED = 'FAILED'
    BLOCKED = 'BLOCKED'

    def __init__(self, entry_bearing, release_bearing, minimum_distance,
                 angular_gain, minimum_angular_speed, maximum_angular_speed,
                 alignment_timeout, command_watchdog):
        self.entry_bearing = float(entry_bearing)
        self.release_bearing = float(release_bearing)
        self.minimum_distance = float(minimum_distance)
        self.angular_gain = float(angular_gain)
        self.minimum_angular_speed = float(minimum_angular_speed)
        self.maximum_angular_speed = float(maximum_angular_speed)
        self.alignment_timeout = float(alignment_timeout)
        self.command_watchdog = float(command_watchdog)
        if not 0.0 < self.release_bearing < self.entry_bearing <= math.pi:
            raise ValueError(
                'bearing thresholds must satisfy 0 < release < entry <= pi')
        self.clear()

    @staticmethod
    def _geometry(goal, robot):
        dx = goal[0] - robot[0]
        dy = goal[1] - robot[1]
        return (math.hypot(dx, dy),
                normalize_angle(math.atan2(dy, dx) - robot[2]))

    def set_goal(self, goal_xy, robot_pose, now):
        self.goal = (float(goal_xy[0]), float(goal_xy[1]))
        self.started = float(now)
        distance, bearing = self._geometry(self.goal, robot_pose)
        self.state = (self.ALIGNING
                      if (distance > self.minimum_distance and
                          abs(bearing) > self.entry_bearing)
                      else self.PASS_THROUGH)

    def clear(self):
        self.goal = None
        self.started = None
        self.state = self.IDLE

    def fail(self):
        self.state = self.FAILED

    def block(self):
        self.state = self.BLOCKED

    def command(self, robot_pose, raw_command, now, raw_stamp):
        if self.state in (self.IDLE, self.FAILED, self.BLOCKED):
            return (0.0, 0.0)
        if float(now) - float(raw_stamp) > self.command_watchdog:
            return (0.0, 0.0)
        if self.state == self.ALIGNING:
            if float(now) - self.started > self.alignment_timeout:
                self.fail()
                return (0.0, 0.0)
            unused_distance, bearing = self._geometry(self.goal, robot_pose)
            if abs(bearing) < self.release_bearing:
                self.state = self.PASS_THROUGH
                return raw_command
            speed = min(self.maximum_angular_speed,
                        max(self.minimum_angular_speed,
                            self.angular_gain * abs(bearing)))
            return (0.0, math.copysign(speed, bearing))
        return raw_command
