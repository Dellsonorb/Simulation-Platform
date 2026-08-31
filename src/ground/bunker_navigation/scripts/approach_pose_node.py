#!/usr/bin/env python
from __future__ import division

import json
import math
import threading

import actionlib
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetPlan, GetPlanRequest
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from bunker_navigation.approach_pose import (ApproachParameters,
                                               candidate_bearing_is_supported,
                                               candidate_is_clear,
                                               costmap_is_ready,
                                               generate_candidates,
                                               normalize_angle,
                                               path_sweep_avoids_target)
from bunker_navigation.costmap_sweep import CostmapGrid, pad_footprint


class ApproachPoseNode(object):
    def __init__(self):
        self.target_frame = rospy.get_param('~target_frame', 'odom')
        self.robot_frame = rospy.get_param('~robot_frame', 'base_link')
        self.costmap_max_age = float(rospy.get_param('~costmap_max_age', 1.0))
        self.costmap_settle_duration = float(rospy.get_param(
            '~costmap_settle_duration', 0.8))
        self.input_wait_timeout = float(rospy.get_param(
            '~input_wait_timeout', 8.0))
        self.plan_tolerance = float(rospy.get_param('~plan_tolerance', 0.05))
        self.yaw_score_weight = float(rospy.get_param(
            '~yaw_score_weight', 0.15))
        self.direct_drive_bearing_limit = math.radians(float(rospy.get_param(
            '~direct_drive_bearing_limit_deg', 30.0)))
        self.heading_gate_entry_bearing = math.radians(float(rospy.get_param(
            '/heading_gate/entry_bearing_deg', 100.0)))
        self.navigation_timeout = float(rospy.get_param(
            '~navigation_timeout', 90.0))
        self.lethal_cost_threshold = int(rospy.get_param(
            '~lethal_cost_threshold', 100))
        self.brick_length = float(rospy.get_param('~brick_length', 0.240))
        self.brick_width = float(rospy.get_param('~brick_width', 0.115))
        self.target_safety_padding = float(rospy.get_param(
            '~target_safety_padding', 0.03))
        self.target_sweep_linear_step = float(rospy.get_param(
            '~target_sweep_linear_step', 0.025))
        self.target_sweep_angular_step = float(rospy.get_param(
            '~target_sweep_angular_step', 0.025))
        self.parameters = ApproachParameters(
            work_distance=float(rospy.get_param('~work_distance', 0.82)),
            arm_offset_x=float(rospy.get_param('~arm_offset_x', 0.15)),
            arm_offset_y=float(rospy.get_param('~arm_offset_y', 0.0)),
            reach_min=float(rospy.get_param('~reach_min', 0.75)),
            reach_max=float(rospy.get_param('~reach_max', 0.88)),
            lateral_limit=float(rospy.get_param('~lateral_limit', 0.12)))
        self.parameters.validate()
        footprint = rospy.get_param(
            '~footprint', [[-0.50, -0.37], [-0.50, 0.37],
                           [0.50, 0.37], [0.50, -0.37]])
        self.footprint = pad_footprint(
            [(float(point[0]), float(point[1])) for point in footprint],
            float(rospy.get_param('~footprint_padding', 0.02)))
        self.listener = tf.TransformListener()
        self.move_base = actionlib.SimpleActionClient('/move_base',
                                                       MoveBaseAction)
        self.make_plan = rospy.ServiceProxy(
            rospy.get_param('~make_plan_service', '/move_base/make_plan'),
            GetPlan)
        self.output = rospy.Publisher(
            rospy.get_param('~output_topic', '/bunker/approach_pose'),
            PoseStamped, queue_size=1, latch=True)
        self.status = rospy.Publisher(
            rospy.get_param('~status_topic', '/bunker/approach_status'),
            String, queue_size=10, latch=True)
        self.markers = rospy.Publisher(
            rospy.get_param('~marker_topic', '/bunker/approach_markers'),
            MarkerArray, queue_size=1, latch=True)
        self.costmap = None
        self.costmap_received_at = None
        self.lock = threading.RLock()
        self.active = False
        rospy.Subscriber(
            rospy.get_param('~costmap_topic',
                            '/move_base/global_costmap/costmap'),
            OccupancyGrid, self.costmap_callback, queue_size=1)
        rospy.Subscriber(
            rospy.get_param('~input_topic', '/known_brick_pose'),
            PoseStamped, self.brick_callback, queue_size=1)
        self.publish_status('READY')

    @staticmethod
    def yaw(pose):
        quaternion = (pose.orientation.x, pose.orientation.y,
                      pose.orientation.z, pose.orientation.w)
        norm = math.sqrt(sum(value * value for value in quaternion))
        if norm <= 1e-9 or not all(not math.isnan(value) and
                                   not math.isinf(value)
                                   for value in quaternion):
            raise ValueError('brick quaternion is invalid')
        return tf.transformations.euler_from_quaternion(quaternion)[2]

    @staticmethod
    def pose_message(candidate, frame, stamp=None):
        message = PoseStamped()
        message.header.frame_id = frame
        message.header.stamp = stamp or rospy.Time.now()
        message.pose.position.x = candidate.x
        message.pose.position.y = candidate.y
        quaternion = tf.transformations.quaternion_from_euler(
            0.0, 0.0, candidate.yaw)
        message.pose.orientation.x = quaternion[0]
        message.pose.orientation.y = quaternion[1]
        message.pose.orientation.z = quaternion[2]
        message.pose.orientation.w = quaternion[3]
        return message

    def publish_status(self, state, reason='', **values):
        result = {'state': state, 'reason': reason,
                  'stamp': rospy.Time.now().to_sec()}
        result.update(values)
        self.status.publish(String(data=json.dumps(result, sort_keys=True)))
        rospy.loginfo('[APPROACH] %s', json.dumps(result, sort_keys=True))

    def costmap_callback(self, message):
        grid = CostmapGrid(
            message.info.width, message.info.height, message.info.resolution,
            message.info.origin.position.x, message.info.origin.position.y,
            message.data)
        with self.lock:
            self.costmap = grid
            self.costmap_received_at = rospy.Time.now()

    def brick_callback(self, message):
        with self.lock:
            if self.active:
                self.publish_status('REJECTED', 'BUSY')
                return
            self.active = True
        worker = threading.Thread(
            target=self.process, args=(message, rospy.Time.now()))
        worker.daemon = True
        worker.start()

    def wait_until_ready(self, input_received_at):
        deadline = rospy.Time.now() + rospy.Duration(self.input_wait_timeout)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            with self.lock:
                now = rospy.Time.now()
                fresh = (self.costmap is not None and costmap_is_ready(
                    self.costmap_received_at.to_sec()
                    if self.costmap_received_at is not None else None,
                    now.to_sec(), input_received_at.to_sec(),
                    self.costmap_max_age, self.costmap_settle_duration))
            if fresh and self.move_base.wait_for_server(rospy.Duration(0.1)):
                try:
                    rospy.wait_for_service(self.make_plan.resolved_name,
                                           timeout=0.1)
                    return True
                except rospy.ROSException:
                    pass
            rospy.rostime.wallsleep(0.05)
        return False

    def robot_pose(self):
        self.listener.waitForTransform(
            self.target_frame, self.robot_frame, rospy.Time(0),
            rospy.Duration(2.0))
        translation, quaternion = self.listener.lookupTransform(
            self.target_frame, self.robot_frame, rospy.Time(0))
        return (translation[0], translation[1],
                tf.transformations.euler_from_quaternion(quaternion)[2])

    def transform_brick(self, message):
        if not message.header.frame_id:
            raise ValueError('brick pose frame is empty')
        if message.header.frame_id == self.target_frame:
            transformed = message
        else:
            self.listener.waitForTransform(
                self.target_frame, message.header.frame_id, rospy.Time(0),
                rospy.Duration(2.0))
            transformed = self.listener.transformPose(self.target_frame,
                                                       message)
        values = (transformed.pose.position.x, transformed.pose.position.y,
                  self.yaw(transformed.pose))
        if not all(not math.isnan(value) and not math.isinf(value)
                   for value in values):
            raise ValueError('brick pose is not finite')
        return transformed, values

    def path_length(self, poses):
        total = 0.0
        for previous, current in zip(poses[:-1], poses[1:]):
            dx = current.pose.position.x - previous.pose.position.x
            dy = current.pose.position.y - previous.pose.position.y
            total += math.hypot(dx, dy)
        return total

    def planned_candidates(self, candidates, current_pose, grid, brick_pose):
        start = self.pose_message(type('Current', (), {
            'x': current_pose[0], 'y': current_pose[1],
            'yaw': current_pose[2]})(), self.target_frame)
        planned = []
        rejected = []
        for candidate in candidates:
            if not candidate_bearing_is_supported(
                    candidate, current_pose,
                    self.direct_drive_bearing_limit,
                    self.heading_gate_entry_bearing):
                rejected.append({'id': candidate.identifier,
                                 'reason': 'DWA_LATERAL_BEARING'})
                continue
            clear, reason = candidate_is_clear(
                grid, self.footprint, candidate, self.lethal_cost_threshold)
            if not clear:
                rejected.append({'id': candidate.identifier,
                                 'reason': reason})
                continue
            goal = self.pose_message(candidate, self.target_frame)
            request = GetPlanRequest(start=start, goal=goal,
                                     tolerance=self.plan_tolerance)
            response = self.make_plan(request)
            if not response.plan.poses:
                rejected.append({'id': candidate.identifier,
                                 'reason': 'NO_GLOBAL_PLAN'})
                continue
            path = tuple((
                item.pose.position.x,
                item.pose.position.y,
                self.yaw(item.pose),
            ) for item in response.plan.poses)
            target_clear, target_reason, checked_poses = \
                path_sweep_avoids_target(
                    path, self.footprint, brick_pose,
                    self.brick_length, self.brick_width,
                    self.target_safety_padding,
                    self.target_sweep_linear_step,
                    self.target_sweep_angular_step)
            if not target_clear:
                rejected.append({
                    'id': candidate.identifier,
                    'reason': target_reason,
                    'target_sweep_checked_poses': checked_poses,
                })
                continue
            length = self.path_length(response.plan.poses)
            score = (length + self.yaw_score_weight * abs(normalize_angle(
                candidate.yaw - current_pose[2])) +
                candidate.priority * 1e-4)
            planned.append((score, length, candidate))
        return sorted(planned, key=lambda item: item[0]), rejected

    def publish_visualization(self, brick, candidate):
        arrow = Marker()
        arrow.header.frame_id = self.target_frame
        arrow.header.stamp = rospy.Time.now()
        arrow.ns = 'approach_pose'
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose = self.pose_message(candidate, self.target_frame).pose
        arrow.scale.x = 0.65
        arrow.scale.y = 0.10
        arrow.scale.z = 0.10
        arrow.color.r = 0.10
        arrow.color.g = 0.95
        arrow.color.b = 0.20
        arrow.color.a = 1.0
        marker = Marker()
        marker.header = arrow.header
        marker.ns = 'known_brick'
        marker.id = 1
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = brick.pose
        marker.scale.x = 0.24
        marker.scale.y = 0.115
        marker.scale.z = 0.053
        marker.color.r = 0.95
        marker.color.g = 0.12
        marker.color.b = 0.05
        marker.color.a = 0.8
        self.markers.publish(MarkerArray(markers=[arrow, marker]))

    def process(self, message, input_received_at):
        try:
            if not self.wait_until_ready(input_received_at):
                self.publish_status('REJECTED', 'NAVIGATION_NOT_READY')
                return
            brick, brick_values = self.transform_brick(message)
            current = self.robot_pose()
            candidates = generate_candidates(
                brick_values, current, self.parameters)
            with self.lock:
                grid = self.costmap
            planned, rejected = self.planned_candidates(
                candidates, current, grid, brick_values)
            if not planned:
                self.publish_status('REJECTED', 'NO_SAFE_CANDIDATE',
                                    rejected_candidates=rejected)
                return
            score, plan_length, selected = planned[0]
            approach = self.pose_message(selected, self.target_frame)
            self.output.publish(approach)
            self.publish_visualization(brick, selected)
            self.publish_status(
                'APPROACH_GENERATED', candidate_id=selected.identifier,
                candidate_count=len(candidates), safe_candidate_count=len(planned),
                plan_length=plan_length, score=score,
                rejected_candidates=rejected)
            goal = MoveBaseGoal(target_pose=approach)
            self.publish_status('NAVIGATING', candidate_id=selected.identifier)
            self.move_base.send_goal(goal)
            if not self.move_base.wait_for_result(
                    rospy.Duration(self.navigation_timeout)):
                self.move_base.cancel_goal()
                self.publish_status('NAVIGATION_FAILED', 'TIMEOUT')
                return
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                self.publish_status('ARRIVED', candidate_id=selected.identifier)
            else:
                self.publish_status('NAVIGATION_FAILED',
                                    'ACTION_STATE_{}'.format(state))
        except Exception as error:
            rospy.logerr('[APPROACH] processing failed: %s', error)
            self.publish_status('REJECTED', 'PROCESSING_ERROR',
                                detail=str(error))
        finally:
            with self.lock:
                self.active = False


if __name__ == '__main__':
    rospy.init_node('approach_pose_generator')
    ApproachPoseNode()
    rospy.spin()
