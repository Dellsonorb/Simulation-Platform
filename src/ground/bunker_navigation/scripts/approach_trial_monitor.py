#!/usr/bin/env python
from __future__ import division

import json
import math
import os
import time

import rospy
import tf
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

from bunker_navigation.approach_metrics import relative_brick_metrics
from bunker_navigation.heading_gate import normalize_angle


TERMINAL_STATES = ('ARRIVED', 'NAVIGATION_FAILED', 'REJECTED')


def as_bool(value):
    return value if isinstance(value, bool) else str(value).lower() == 'true'


class ApproachTrialMonitor(object):
    def __init__(self):
        self.scenario_id = rospy.get_param('~scenario_id')
        self.expect_generation = as_bool(rospy.get_param(
            '~expect_generation'))
        self.small_obstacle = as_bool(rospy.get_param(
            '~small_obstacle', False))
        self.result_file = rospy.get_param('~result_file')
        self.timeout = float(rospy.get_param('~timeout', 100.0))
        self.work_distance = float(rospy.get_param('~work_distance', 0.82))
        self.arm_offset_x = float(rospy.get_param('~arm_offset_x', 0.15))
        self.arm_offset_y = float(rospy.get_param('~arm_offset_y', 0.0))
        self.distance_tolerance = float(rospy.get_param(
            '~distance_tolerance', 0.07))
        self.facing_tolerance = float(rospy.get_param(
            '~facing_tolerance', 0.07))
        self.goal_xy_tolerance = float(rospy.get_param(
            '~goal_xy_tolerance', 0.06))
        self.goal_yaw_tolerance = float(rospy.get_param(
            '~goal_yaw_tolerance', 0.06))
        self.listener = tf.TransformListener()
        self.get_state = rospy.ServiceProxy('/gazebo/get_model_state',
                                            GetModelState)
        self.known_brick = rospy.Publisher('/known_brick_pose', PoseStamped,
                                           queue_size=1, latch=True)
        self.approach = None
        self.latest_status = None
        self.status_history = []
        self.generated_status = None
        self.costmap_received_at = None
        self.latest_cmd = Twist()
        self.gate_states = []
        rospy.Subscriber('/bunker/approach_pose', PoseStamped,
                         self.approach_callback, queue_size=1)
        rospy.Subscriber('/bunker/approach_status', String,
                         self.status_callback, queue_size=20)
        rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid,
                         self.costmap_callback, queue_size=1)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_callback, queue_size=1)
        rospy.Subscriber('/heading_gate/state', String,
                         self.gate_callback, queue_size=20)

    def approach_callback(self, message):
        self.approach = message

    def status_callback(self, message):
        try:
            status = json.loads(message.data)
        except ValueError:
            return
        self.latest_status = status
        state = status.get('state')
        if not self.status_history or self.status_history[-1] != state:
            self.status_history.append(state)
        if state == 'APPROACH_GENERATED':
            self.generated_status = status

    def costmap_callback(self, unused_message):
        self.costmap_received_at = rospy.Time.now().to_sec()

    def cmd_callback(self, message):
        self.latest_cmd = message

    def gate_callback(self, message):
        if not self.gate_states or self.gate_states[-1] != message.data:
            self.gate_states.append(message.data)

    @staticmethod
    def yaw(pose):
        quaternion = (pose.orientation.x, pose.orientation.y,
                      pose.orientation.z, pose.orientation.w)
        return tf.transformations.euler_from_quaternion(quaternion)[2]

    def wait_for_model(self, name, timeout=25.0):
        deadline = time.time() + timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            state = self.get_state(name, 'world')
            if state.success:
                return state
            rospy.rostime.wallsleep(0.1)
        raise RuntimeError('Gazebo model unavailable: ' + name)

    def wait_until_ready(self, timeout=30.0):
        deadline = time.time() + timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            fresh = (self.costmap_received_at is not None and
                     rospy.Time.now().to_sec() - self.costmap_received_at < 0.5)
            ready = (self.latest_status is not None and
                     self.latest_status.get('state') == 'READY')
            if fresh and ready:
                return True
            rospy.rostime.wallsleep(0.05)
        return False

    def write(self, result):
        directory = os.path.dirname(self.result_file)
        if directory and not os.path.isdir(directory):
            os.makedirs(directory)
        temporary = self.result_file + '.tmp'
        with open(temporary, 'w') as stream:
            json.dump(result, stream, indent=2, sort_keys=True)
            stream.write('\n')
        os.rename(temporary, self.result_file)
        rospy.loginfo('[APPROACH_TRIAL] %s', json.dumps(result,
                                                        sort_keys=True))

    def run(self):
        started = time.time()
        try:
            rospy.wait_for_service('/gazebo/get_model_state', timeout=25.0)
            brick_initial = self.wait_for_model('brick')
            if self.small_obstacle:
                self.wait_for_model('approach_obstacle')
            if not self.wait_until_ready():
                raise RuntimeError('approach node or global costmap unavailable')
            # Allow laser marking and the physical brick to settle before the
            # known pose is sampled and published.
            rospy.rostime.wallsleep(3.0)
            brick_initial = self.get_state('brick', 'world')
            known = PoseStamped()
            known.header.frame_id = 'odom'
            known.header.stamp = rospy.Time.now()
            known.pose = brick_initial.pose
            navigation_started = time.time()
            self.known_brick.publish(known)
            deadline = time.time() + self.timeout
            while not rospy.is_shutdown() and time.time() < deadline:
                if (self.latest_status is not None and
                        self.latest_status.get('state') in TERMINAL_STATES):
                    break
                rospy.rostime.wallsleep(0.05)
            terminal = self.latest_status or {'state': 'TIMEOUT',
                                              'reason': 'NO_STATUS'}
            rospy.rostime.wallsleep(0.35)
            bunker = self.get_state('bunker_aubo', 'world')
            brick = self.get_state('brick', 'world')
            base = (bunker.pose.position.x, bunker.pose.position.y,
                    self.yaw(bunker.pose))
            brick_pose = (brick.pose.position.x, brick.pose.position.y,
                          self.yaw(brick.pose))
            generation_success = (self.approach is not None and
                                  self.generated_status is not None)
            navigation_started_seen = 'NAVIGATING' in self.status_history
            navigation_success = terminal.get('state') == 'ARRIVED'
            metrics = None
            goal_xy_error = None
            goal_yaw_error = None
            if generation_success:
                metrics = relative_brick_metrics(
                    base, brick_pose, self.arm_offset_x, self.arm_offset_y,
                    self.work_distance)
                target_yaw = self.yaw(self.approach.pose)
                goal_xy_error = math.hypot(
                    base[0] - self.approach.pose.position.x,
                    base[1] - self.approach.pose.position.y)
                goal_yaw_error = abs(normalize_angle(base[2] - target_yaw))
            stopped = (abs(self.latest_cmd.linear.x) <= 0.01 and
                       abs(self.latest_cmd.angular.z) <= 0.01)
            if self.expect_generation:
                success = bool(
                    generation_success and navigation_success and metrics and
                    metrics['distance_error'] <= self.distance_tolerance and
                    metrics['facing_error'] <= self.facing_tolerance and
                    goal_xy_error <= self.goal_xy_tolerance and
                    goal_yaw_error <= self.goal_yaw_tolerance and stopped)
            else:
                success = bool(
                    not generation_success and not navigation_started_seen and
                    terminal.get('state') == 'REJECTED' and
                    terminal.get('reason') == 'NO_SAFE_CANDIDATE' and stopped)
            failure_type = ''
            if not success:
                if self.expect_generation and not generation_success:
                    failure_type = 'generation_failed'
                elif self.expect_generation and not navigation_success:
                    failure_type = 'navigation_failed'
                elif not self.expect_generation and generation_success:
                    failure_type = 'unsafe_candidate_generated'
                elif not stopped:
                    failure_type = 'not_stopped'
                else:
                    failure_type = 'accuracy_failed'
            result = {
                'scenario_id': self.scenario_id,
                'expect_generation': self.expect_generation,
                'generation_success': generation_success,
                'navigation_started': navigation_started_seen,
                'navigation_success': navigation_success,
                'success': success,
                'failure_type': failure_type,
                'terminal_status': terminal,
                'status_history': self.status_history,
                'candidate_id': (self.generated_status or {}).get(
                    'candidate_id'),
                'safe_candidate_count': (self.generated_status or {}).get(
                    'safe_candidate_count'),
                'base_actual': {'x': base[0], 'y': base[1], 'yaw': base[2]},
                'brick_actual': {'x': brick_pose[0], 'y': brick_pose[1],
                                 'yaw': brick_pose[2]},
                'approach_target': ({
                    'x': self.approach.pose.position.x,
                    'y': self.approach.pose.position.y,
                    'yaw': self.yaw(self.approach.pose)}
                    if self.approach is not None else None),
                'center_distance': (metrics or {}).get('center_distance'),
                'distance_error': (metrics or {}).get('distance_error'),
                'facing_error': (metrics or {}).get('facing_error'),
                'arm_forward': (metrics or {}).get('arm_forward'),
                'arm_lateral': (metrics or {}).get('arm_lateral'),
                'arm_workspace_error': (metrics or {}).get(
                    'arm_workspace_error'),
                'goal_xy_error': goal_xy_error,
                'goal_yaw_error': goal_yaw_error,
                'linear_speed': abs(self.latest_cmd.linear.x),
                'angular_speed': abs(self.latest_cmd.angular.z),
                'stopped': stopped,
                'heading_gate_states': self.gate_states,
                'duration': time.time() - navigation_started,
                'wall_duration': time.time() - started,
            }
            self.write(result)
            return 0 if success else 1
        except Exception as error:
            self.write({
                'scenario_id': self.scenario_id,
                'expect_generation': self.expect_generation,
                'generation_success': False,
                'navigation_success': False,
                'success': False,
                'failure_type': 'harness_failed',
                'failure_detail': str(error),
                'distance_error': None,
                'facing_error': None,
                'duration': time.time() - started})
            rospy.logerr('[APPROACH_TRIAL] %s', error)
            return 1


if __name__ == '__main__':
    rospy.init_node('approach_trial_monitor')
    raise SystemExit(ApproachTrialMonitor().run())
