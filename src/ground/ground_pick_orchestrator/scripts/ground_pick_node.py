#!/usr/bin/env python
from __future__ import division

import json
import subprocess
import sys
import threading
import time

import actionlib
import moveit_commander
import rospy
from actionlib_msgs.msg import GoalID
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal)
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from ground_pick_orchestrator.mission_state import MissionStateMachine
from ground_pick_orchestrator.observation_planning import (
    validated_observation_trajectory)
from ground_pick_orchestrator.safety_guards import (motion_is_stopped,
                                                     joint_window_is_settled,
                                                     refined_pose_is_valid)


class GroundPickOrchestrator(object):
    def __init__(self):
        self.lock = threading.RLock()
        self.required_frame = rospy.get_param(
            '~required_pose_frame', 'aubo_i5_base_link')
        self.linear_stop_threshold = float(rospy.get_param(
            '~linear_stop_threshold', 0.01))
        self.angular_stop_threshold = float(rospy.get_param(
            '~angular_stop_threshold', 0.02))
        self.stop_timeout = float(rospy.get_param('~stop_timeout', 8.0))
        self.observation_timeout = float(rospy.get_param(
            '~observation_timeout', 30.0))
        self.perception_timeout = float(rospy.get_param(
            '~perception_timeout', 20.0))
        self.navigation_timeout = float(rospy.get_param(
            '~navigation_timeout', 100.0))
        self.pick_timeout = float(rospy.get_param('~pick_timeout', 90.0))
        self.mission_timeout = float(rospy.get_param(
            '~mission_timeout', 190.0))
        self.minimum_lift = float(rospy.get_param('~minimum_lift', 0.10))
        self.lift_verify_delay = float(rospy.get_param(
            '~lift_verify_delay', 1.0))
        self.observation_duration = float(rospy.get_param(
            '~observation_duration', 5.0))
        self.observation_goal_tolerance = float(rospy.get_param(
            '~observation_goal_tolerance', 0.08))
        self.observation_position_delta_tolerance = float(rospy.get_param(
            '~observation_position_delta_tolerance', 0.005))
        self.observation_settle_duration = float(rospy.get_param(
            '~observation_settle_duration', 0.5))
        self.mobile_handoff_timeout = float(rospy.get_param(
            '~mobile_handoff_timeout', 8.0))
        self.controller_start_timeout = float(rospy.get_param(
            '~controller_start_timeout', 15.0))
        self.observation_planning_time = float(rospy.get_param(
            '~observation_planning_time', 6.0))
        self.observation_planning_attempts = int(rospy.get_param(
            '~observation_planning_attempts', 5))
        self.observation_velocity_scaling = float(rospy.get_param(
            '~observation_velocity_scaling', 0.20))
        self.observation_acceleration_scaling = float(rospy.get_param(
            '~observation_acceleration_scaling', 0.20))
        self.observation_plan_endpoint_tolerance = float(rospy.get_param(
            '~observation_plan_endpoint_tolerance', 0.01))
        self.observation_trajectory_timeout_margin = float(rospy.get_param(
            '~observation_trajectory_timeout_margin', 5.0))
        self.observation_ground_frame = rospy.get_param(
            '~observation_ground_frame', 'aubo_i5_base_link')
        self.observation_ground_center = list(rospy.get_param(
            '~observation_ground_center', [2.0, 0.0, -0.532]))
        self.observation_ground_size = list(rospy.get_param(
            '~observation_ground_size', [3.2, 4.0, 0.10]))
        self.brick_model = rospy.get_param('~brick_model', 'brick')
        self.machine = MissionStateMachine(
            rospy.get_param('~stop_settle_time', 1.0), self.required_frame)

        self.status_pub = rospy.Publisher(
            rospy.get_param('~status_topic', '/ground_pick/status'),
            String, queue_size=20, latch=True)
        self.approach_input_pub = rospy.Publisher(
            rospy.get_param('~approach_input_topic', '/known_brick_pose'),
            PoseStamped, queue_size=1, latch=True)
        self.legacy_pick_pose_pub = rospy.Publisher(
            rospy.get_param('~legacy_pick_pose_topic', '/brick_pose'),
            PoseStamped, queue_size=1, latch=True)
        self.approved_pose_pub = rospy.Publisher(
            rospy.get_param('~approved_pose_topic',
                            '/ground_pick/approved_brick_pose'),
            PoseStamped, queue_size=1, latch=True)
        self.zero_command_pub = rospy.Publisher('/cmd_vel', Twist,
                                                queue_size=1)
        self.mobile_handoff_pub = rospy.Publisher(
            rospy.get_param('~mobile_handoff_command_topic',
                            '/ground_pick/mobile_manipulation/command'),
            String, queue_size=1, latch=True)
        self.move_base = actionlib.SimpleActionClient('/move_base',
                                                       MoveBaseAction)
        self.arm_controller = actionlib.SimpleActionClient(
            '/arm_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction)
        self.gripper_controller = actionlib.SimpleActionClient(
            '/gripper_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction)
        self.cancel_publishers = [
            rospy.Publisher(topic, GoalID, queue_size=1)
            for topic in ('/move_group/cancel',
                          '/arm_controller/follow_joint_trajectory/cancel',
                          '/gripper_controller/follow_joint_trajectory/cancel')]
        self.get_model_state = rospy.ServiceProxy(
            '/gazebo/get_model_state', GetModelState)
        self.observation_joint_names = rospy.get_param(
            '/ground_pick_observation/joint_names')
        self.observation_positions = rospy.get_param(
            '/ground_pick_observation/positions')

        self.latest_odom = None
        self.latest_command = None
        self.latest_odom_at = None
        self.latest_command_at = None
        self.latest_joint_state = None
        self.latest_joint_state_wall = None
        self.mobile_manipulation_state = ''
        self.pending_refined_pose = None
        self.observation_gate_verified = False
        self.refine_gate_process = None
        self.controller_spawner_process = None
        self.state_entered_wall = time.time()
        self.mission_started_wall = None
        self.lift_log_wall = None
        self.initial_brick_height = None
        self.final_brick_height = None
        self.fresh_pose_after_stop = False
        self.last_published_state = None
        self.refine_worker = None
        self.manipulation_worker = None
        self.lift_worker = None
        self.lift_verification_started = False
        self.safety_stop_started = False
        self.observation_move_group = None
        self.observation_scene = None

        rospy.Subscriber(
            rospy.get_param('~input_topic',
                            '/ground_pick/known_brick_pose'),
            PoseStamped, self.input_callback, queue_size=1)
        rospy.Subscriber(
            rospy.get_param('~approach_status_topic',
                            '/bunker/approach_status'),
            String, self.approach_status_callback, queue_size=20)
        rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=10)
        rospy.Subscriber('/cmd_vel', Twist, self.command_callback,
                         queue_size=10)
        rospy.Subscriber('/joint_states', JointState,
                         self.joint_state_callback, queue_size=10)
        rospy.Subscriber(
            rospy.get_param('~mobile_handoff_status_topic',
                            '/ground_pick/mobile_manipulation/status'),
            String, self.mobile_manipulation_status_callback, queue_size=10)
        rospy.Subscriber(
            rospy.get_param('~refine_output_topic',
                            '/ground_pick/refined_brick_pose'),
            PoseStamped, self.refined_pose_callback, queue_size=1)
        rospy.Subscriber(
            rospy.get_param('~refine_status_topic',
                            '/ground_pick/refine_status'),
            String, self.refine_status_callback, queue_size=10)
        rospy.Subscriber('/rosout_agg', Log, self.log_callback,
                         queue_size=100)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.timer_callback)
        rospy.on_shutdown(self.shutdown)
        self.publish_status(force=True)

    def status_payload(self):
        return {
            'state': self.machine.state,
            'failure_type': self.machine.failure_type,
            'failure_detail': self.machine.failure_detail,
            'navigation_success': self.machine.navigation_success,
            'perception_success': self.machine.perception_success,
            'planning_success': self.machine.planning_success,
            'grasp_success': self.machine.grasp_success,
            'pick_started': self.machine.pick_started,
            'end_to_end_success': self.machine.end_to_end_success,
            'fresh_pose_after_stop': self.fresh_pose_after_stop,
            'initial_brick_z': self.initial_brick_height,
            'final_brick_z': self.final_brick_height,
            'stamp': rospy.Time.now().to_sec(),
        }

    def publish_status(self, force=False):
        state = self.machine.state
        if force or state != self.last_published_state:
            payload = self.status_payload()
            self.status_pub.publish(String(data=json.dumps(payload,
                                                           sort_keys=True)))
            rospy.loginfo('[GROUND_PICK] %s', json.dumps(payload,
                                                        sort_keys=True))
            self.last_published_state = state
            self.state_entered_wall = time.time()

    def brick_height(self):
        try:
            result = self.get_model_state(self.brick_model, 'world')
            if result.success:
                return result.pose.position.z
        except rospy.ServiceException:
            pass
        return None

    def start_daemon_worker(self, target, name, args=()):
        worker = threading.Thread(target=target, name=name, args=args)
        worker.daemon = True
        worker.start()
        return worker

    def mission_active(self, expected_state=None):
        with self.lock:
            if self.machine.terminal:
                return False
            return expected_state is None or self.machine.state == expected_state

    def fail_from_worker(self, category, detail):
        failed = False
        with self.lock:
            if self.machine.fail(category, detail):
                self.publish_status(force=True)
                failed = True
        if failed:
            self.safety_stop()

    def initialize_mission(self, request):
        initial_height = self.brick_height()
        with self.lock:
            if self.machine.terminal or self.machine.state != 'WAITING_APPROACH':
                return
            self.initial_brick_height = initial_height
            self.approach_input_pub.publish(request)

    def input_callback(self, message):
        with self.lock:
            if self.machine.state != 'IDLE':
                rospy.logwarn('[GROUND_PICK] ignoring input while state=%s',
                              self.machine.state)
                return
            now = rospy.Time.now()
            request = PoseStamped()
            request.header = message.header
            request.header.stamp = now
            request.pose = message.pose
            self.mission_started_wall = time.time()
            self.machine.start(now.to_sec())
            rospy.set_param('/brick_pick_demo/success', False)
            self.publish_status(force=True)
            self.start_daemon_worker(
                self.initialize_mission, name='ground_pick_initialize',
                args=(request,))

    def approach_status_callback(self, message):
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError):
            return
        with self.lock:
            if self.machine.state == 'IDLE' or self.machine.terminal:
                return
            previous = self.machine.state
            self.machine.handle_approach(
                status.get('state', ''), status.get('reason', ''),
                rospy.Time.now().to_sec())
            if self.machine.state != previous:
                self.publish_status(force=True)
            if self.machine.state == 'FAILED':
                self.safety_stop()

    def odom_callback(self, message):
        with self.lock:
            self.latest_odom = (
                message.twist.twist.linear.x,
                message.twist.twist.linear.y,
                message.twist.twist.angular.z)
            self.latest_odom_at = rospy.Time.now().to_sec()

    def command_callback(self, message):
        with self.lock:
            self.latest_command = (message.linear.x, message.angular.z)
            self.latest_command_at = rospy.Time.now().to_sec()

    def joint_state_callback(self, message):
        with self.lock:
            self.latest_joint_state = message
            self.latest_joint_state_wall = time.time()

    def mobile_manipulation_status_callback(self, message):
        with self.lock:
            self.mobile_manipulation_state = message.data

    def wait_mobile_manipulation_state(self, expected, timeout=None):
        timeout = self.mobile_handoff_timeout if timeout is None else timeout
        deadline = time.time() + timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            with self.lock:
                state = self.mobile_manipulation_state
                terminal = self.machine.terminal
            if terminal:
                return False
            if state == expected:
                return True
            if state == 'FAILED':
                rospy.logerr('[GROUND_PICK] mobile handoff reported FAILED')
                return False
            rospy.rostime.wallsleep(0.02)
        rospy.logerr('[GROUND_PICK] timed out waiting for mobile state %s '
                     '(last=%s)', expected, state)
        return False

    def schedule_refine(self):
        now = rospy.Time.now().to_sec()
        if not self.machine.refine_started(now):
            return
        self.publish_status(force=True)
        self.refine_worker = self.start_daemon_worker(
            self.refine_worker_main, name='ground_pick_refine')

    def refine_worker_main(self):
        if not self.prepare_mobile_manipulation():
            self.fail_from_worker('manipulation_mode_failed',
                                  'anchored_release_or_controller_failed')
            return
        if not self.mission_active('MOVING_OBSERVATION'):
            return
        if not self.move_to_observation_trajectory():
            self.fail_from_worker('observation_failed',
                                  'trajectory_observation_failed')
            return
        if not self.mission_active('MOVING_OBSERVATION'):
            return
        if not self.activate_manipulator_gravity():
            self.fail_from_worker('manipulation_mode_failed',
                                  'gravity_activation_or_settle_failed')
            return
        try:
            process = subprocess.Popen([
                'rosrun', 'brick_visual_pick', 'brick_pose_gate.py',
                '__name:=ground_pick_refine_gate'])
        except OSError as error:
            self.fail_from_worker('observation_failed', str(error))
            return
        with self.lock:
            if self.machine.terminal or self.machine.state != \
                    'MOVING_OBSERVATION':
                process.terminate()
                return
            self.refine_gate_process = process
            self.machine.observation_finished(
                True, 'trajectory_observation_complete')
            self.publish_status(force=True)

    def request_manipulation_controllers(self):
        if self.arm_controller.wait_for_server(rospy.Duration(0.2)):
            return True
        if self.controller_spawner_process is None:
            self.controller_spawner_process = subprocess.Popen([
                'rosrun', 'controller_manager', 'spawner',
                'joint_state_controller', 'arm_controller',
                'gripper_controller'])
        return self.controller_spawner_process.poll() is None

    def wait_manipulation_controllers(self):
        if not self.arm_controller.wait_for_server(
                rospy.Duration(self.controller_start_timeout)):
            rospy.logerr('[GROUND_PICK] AUBO controller startup timed out')
            return False
        if not self.gripper_controller.wait_for_server(
                rospy.Duration(self.controller_start_timeout)):
            rospy.logerr('[GROUND_PICK] AG95 controller startup timed out')
            return False
        try:
            rospy.wait_for_message('/joint_states', JointState,
                                   timeout=self.controller_start_timeout)
        except rospy.ROSException:
            rospy.logerr('[GROUND_PICK] joint-state stream startup timed out')
            return False
        return (self.controller_spawner_process is None or
                self.controller_spawner_process.poll() is None)

    def prepare_mobile_manipulation(self):
        """Release the anchored mobile model without deleting or respawning it."""
        self.mobile_handoff_pub.publish(String(data='RELEASE'))
        if not self.wait_mobile_manipulation_state('ANCHORED_RELEASED'):
            return False
        if not self.request_manipulation_controllers():
            return False
        if not self.wait_manipulation_controllers():
            return False
        return True

    def activate_manipulator_gravity(self):
        self.mobile_handoff_pub.publish(String(data='ENABLE_GRAVITY'))
        if not self.wait_mobile_manipulation_state('ACTIVE_READY'):
            return False
        return self.observation_target_is_settled()

    def observation_target_is_settled(self):
        deadline = time.time() + self.observation_duration + 5.0
        samples = []
        last_stamp = None
        targets = dict(zip(self.observation_joint_names,
                           self.observation_positions))
        maximum_reported_velocity = 0.0
        while not rospy.is_shutdown() and time.time() < deadline:
            with self.lock:
                message = self.latest_joint_state
                received_wall = self.latest_joint_state_wall
            if message is None or received_wall is None or \
                    time.time() - received_wall > 0.5:
                rospy.rostime.wallsleep(0.02)
                continue
            stamp = message.header.stamp.to_sec()
            if stamp == last_stamp:
                rospy.rostime.wallsleep(0.02)
                continue
            last_stamp = stamp
            positions = dict(zip(message.name, message.position))
            velocities = dict(zip(message.name, message.velocity))
            if not all(name in positions for name in self.observation_joint_names):
                rospy.rostime.wallsleep(0.02)
                continue
            samples.append((stamp, positions))
            samples = [sample for sample in samples
                       if stamp - sample[0] <=
                       self.observation_settle_duration + 0.05]
            if velocities:
                maximum_reported_velocity = max(
                    maximum_reported_velocity,
                    max(abs(velocities.get(name, 0.0))
                        for name in self.observation_joint_names))
            if joint_window_is_settled(
                    samples, targets, self.observation_goal_tolerance,
                    self.observation_position_delta_tolerance,
                    self.observation_settle_duration):
                max_error = max(abs(positions[name] - target)
                                for name, target in targets.items())
                rospy.loginfo('[OBSERVATION_TRAJECTORY] settled '
                              'max_error=%.6f position_window<=%.6f '
                              'raw_velocity_peak=%.6f', max_error,
                              self.observation_position_delta_tolerance,
                              maximum_reported_velocity)
                return True
            rospy.rostime.wallsleep(0.02)
        rospy.logerr('[OBSERVATION_TRAJECTORY] target did not settle')
        return False

    def move_to_observation_trajectory(self):
        if len(self.observation_joint_names) != 6 or \
                len(self.observation_positions) != 6:
            rospy.logerr('[GROUND_PICK] invalid observation joint target')
            return False
        try:
            if self.observation_move_group is None:
                self.observation_scene = moveit_commander.PlanningSceneInterface(
                    synchronous=True)
                ground = PoseStamped()
                ground.header.frame_id = self.observation_ground_frame
                ground.pose.orientation.w = 1.0
                (ground.pose.position.x, ground.pose.position.y,
                 ground.pose.position.z) = self.observation_ground_center
                self.observation_scene.add_box(
                    'ground_pick_forward_ground_guard', ground,
                    size=tuple(self.observation_ground_size))
                if 'ground_pick_forward_ground_guard' not in \
                        self.observation_scene.get_known_object_names():
                    rospy.logerr('[OBSERVATION_MOVEIT] ground guard was not '
                                 'accepted by the planning scene')
                    return False
                self.observation_move_group = \
                    moveit_commander.MoveGroupCommander(
                        'manipulator', wait_for_servers=
                        self.controller_start_timeout)
                self.observation_move_group.set_planning_time(
                    self.observation_planning_time)
                self.observation_move_group.set_num_planning_attempts(
                    self.observation_planning_attempts)
                self.observation_move_group.set_max_velocity_scaling_factor(
                    self.observation_velocity_scaling)
                self.observation_move_group.set_max_acceleration_scaling_factor(
                    self.observation_acceleration_scaling)
            group = self.observation_move_group
            group.set_start_state_to_current_state()
            group.set_joint_value_target(dict(zip(
                self.observation_joint_names, self.observation_positions)))
            plan = group.plan()
            trajectory, reason, planned_duration = \
                validated_observation_trajectory(
                    plan, self.observation_joint_names,
                    self.observation_positions,
                    self.observation_plan_endpoint_tolerance)
            if trajectory is None:
                rospy.logerr('[OBSERVATION_MOVEIT] %s', reason)
                group.stop()
                return False
            goal = FollowJointTrajectoryGoal()
            goal.trajectory = trajectory
            goal.trajectory.header.stamp = rospy.Time.now() + \
                rospy.Duration(0.1)
            goal.goal_time_tolerance = rospy.Duration(2.0)
            rospy.loginfo('[OBSERVATION_MOVEIT] planned points=%d '
                          'duration=%.3f target=%s',
                          len(trajectory.points), planned_duration,
                          self.observation_positions)
            self.arm_controller.send_goal(goal)
            if not self.arm_controller.wait_for_result(rospy.Duration(
                    planned_duration +
                    self.observation_trajectory_timeout_margin)):
                self.arm_controller.cancel_goal()
                group.stop()
                rospy.logerr('[OBSERVATION_MOVEIT] action timeout')
                return False
            if self.arm_controller.get_state() != GoalStatus.SUCCEEDED:
                result = self.arm_controller.get_result()
                error_code = getattr(result, 'error_code', None)
                error_string = getattr(result, 'error_string', '')
                rospy.logerr('[OBSERVATION_MOVEIT] action failed state=%d '
                             'error_code=%s error_string=%s',
                             self.arm_controller.get_state(), error_code,
                             error_string)
                group.stop()
                return False
            group.stop()
            return self.observation_target_is_settled()
        except Exception as error:
            rospy.logerr('[OBSERVATION_MOVEIT] planning exception: %s', error)
            if self.observation_move_group is not None:
                self.observation_move_group.stop()
            return False

    def pose_values(self, message):
        pose = message.pose
        return (
            message.header.stamp.to_sec(), pose.position.x, pose.position.y,
            pose.position.z, pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w)

    def refined_pose_callback(self, message):
        with self.lock:
            if self.machine.terminal or self.machine.state not in (
                    'MOVING_OBSERVATION', 'REFINING'):
                return
            if not refined_pose_is_valid(self.pose_values(message)):
                self.machine.fail('invalid_refined_pose',
                                  'non_finite_or_invalid_quaternion')
                self.publish_status(force=True)
                self.safety_stop()
                return
            self.pending_refined_pose = message
            if self.machine.state == 'REFINING':
                self.begin_manipulation_handoff()

    def begin_manipulation_handoff(self):
        message = self.pending_refined_pose
        if message is None:
            return
        if not self.machine.accept_refined_pose(
                message.header.stamp.to_sec(), message.header.frame_id):
            self.publish_status(force=True)
            self.safety_stop()
            return
        self.fresh_pose_after_stop = (
            self.machine.navigation_arrived_at is not None and
            message.header.stamp.to_sec() >
            self.machine.navigation_arrived_at)
        if not self.fresh_pose_after_stop:
            self.machine.fail('stale_refined_pose',
                              'pose_not_after_navigation_stop')
            self.publish_status(force=True)
            self.safety_stop()
            return
        self.publish_status(force=True)
        self.manipulation_worker = self.start_daemon_worker(
            self.manipulation_worker_main,
            name='ground_pick_manipulation', args=(message,))

    def manipulation_worker_main(self, message):
        if not self.wait_manipulation_controllers():
            self.fail_from_worker('controller_start_failed',
                                  'arm_action_server_unavailable')
            return
        with self.lock:
            if self.machine.terminal or self.machine.state != 'PICKING':
                return
            if not self.machine.begin_pick_execution():
                return
            # This is the only publication handoff to the frozen V0.2 pick.
            # It occurs after the fixed-base model and controllers are ready.
            self.legacy_pick_pose_pub.publish(message)
            self.approved_pose_pub.publish(message)
            self.publish_status(force=True)

    def refine_status_callback(self, message):
        with self.lock:
            if self.machine.state not in ('MOVING_OBSERVATION', 'REFINING'):
                return
            if message.data in ('WAITING_ESTIMATE', 'POSE_PUBLISHED'):
                self.observation_gate_verified = True
            previous = self.machine.state
            self.machine.handle_refine_status(message.data)
            if self.machine.state != previous:
                self.publish_status(force=True)
            if self.machine.state == 'FAILED':
                self.safety_stop()

    def log_callback(self, message):
        if message.name != '/brick_pick_node':
            return
        with self.lock:
            previous = self.machine.state
            self.machine.handle_pick_log(message.msg)
            if self.machine.state == 'VERIFYING_LIFT' and previous != \
                    'VERIFYING_LIFT':
                self.lift_log_wall = time.time()
            if self.machine.state != previous:
                self.publish_status(force=True)
            elif self.machine.planning_success or self.machine.grasp_success:
                self.publish_status(force=True)
            if self.machine.state == 'FAILED':
                self.safety_stop()

    def stopped_sample(self, now):
        arrived = self.machine.navigation_arrived_at
        if arrived is None or self.latest_odom_at is None or \
                self.latest_command_at is None:
            return False
        if self.latest_odom_at < arrived or self.latest_command_at < arrived:
            return False
        return motion_is_stopped(
            self.latest_odom, self.latest_command,
            self.linear_stop_threshold, self.angular_stop_threshold)

    def fail_timeout(self, category, detail):
        if self.machine.fail(category, detail):
            self.publish_status(force=True)
            self.safety_stop()

    def lift_verification_worker_main(self):
        final_height = self.brick_height()
        with self.lock:
            if self.machine.terminal or self.machine.state != 'VERIFYING_LIFT':
                return
            self.final_brick_height = final_height
            lifted = (
                self.initial_brick_height is not None and
                self.final_brick_height is not None and
                self.final_brick_height - self.initial_brick_height >=
                self.minimum_lift)
            self.machine.verify_lift(lifted)
            self.publish_status(force=True)
            failed = self.machine.state == 'FAILED'
        if failed:
            self.safety_stop()
        else:
            self.zero_command_pub.publish(Twist())
            self.start_daemon_worker(
                self.publish_zero_command_burst,
                name='ground_pick_success_zero_burst')

    def publish_zero_command_burst(self):
        for _unused in range(3):
            if rospy.is_shutdown():
                return
            self.zero_command_pub.publish(Twist())
            rospy.rostime.wallsleep(0.1)

    def timer_callback(self, _event):
        with self.lock:
            if self.machine.state in ('IDLE', 'SUCCEEDED', 'FAILED'):
                return
            now_wall = time.time()
            if (self.mission_started_wall is not None and
                    now_wall - self.mission_started_wall >
                    self.mission_timeout):
                self.fail_timeout('mission_timeout', 'total_timeout')
                return
            elapsed = now_wall - self.state_entered_wall
            if self.machine.state in ('WAITING_APPROACH', 'NAVIGATING'):
                if elapsed > self.navigation_timeout:
                    self.fail_timeout('navigation_failed', 'orchestrator_timeout')
            elif self.machine.state == 'VERIFYING_STOP':
                stopped = self.stopped_sample(rospy.Time.now().to_sec())
                if self.machine.handle_stop_sample(stopped,
                                                   rospy.Time.now().to_sec()):
                    self.schedule_refine()
                elif elapsed > self.stop_timeout:
                    self.fail_timeout('stop_verification_failed',
                                      'base_not_stationary')
            elif self.machine.state == 'MOVING_OBSERVATION':
                if elapsed > self.observation_timeout:
                    self.fail_timeout('observation_failed', 'timeout')
            elif self.machine.state == 'REFINING':
                if self.pending_refined_pose is not None:
                    self.begin_manipulation_handoff()
                elif elapsed > self.perception_timeout:
                    self.fail_timeout('perception_rejected',
                                      'orchestrator_timeout')
            elif self.machine.state == 'PICKING':
                if elapsed > self.pick_timeout:
                    self.fail_timeout('manipulation_timeout', 'pick_timeout')
            elif self.machine.state == 'VERIFYING_LIFT':
                if (self.lift_log_wall is not None and
                        now_wall - self.lift_log_wall >=
                        self.lift_verify_delay and
                        not self.lift_verification_started):
                    self.lift_verification_started = True
                    self.lift_worker = self.start_daemon_worker(
                        self.lift_verification_worker_main,
                        name='ground_pick_lift_verification')

    def safety_stop(self):
        with self.lock:
            if self.safety_stop_started:
                return
            self.safety_stop_started = True
        self.zero_command_pub.publish(Twist())
        self.move_base.cancel_all_goals()
        cancel = GoalID(stamp=rospy.Time.now())
        for publisher in self.cancel_publishers:
            publisher.publish(cancel)
        self.stop_children()
        self.start_daemon_worker(
            self.publish_zero_command_burst,
            name='ground_pick_safety_zero_burst')

    def stop_children(self):
        for process in (self.refine_gate_process,):
            if process is not None and process.poll() is None:
                process.terminate()

    def shutdown(self):
        with self.lock:
            self.stop_children()


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ground_pick_orchestrator')
    try:
        GroundPickOrchestrator()
        rospy.spin()
    finally:
        moveit_commander.roscpp_shutdown()
