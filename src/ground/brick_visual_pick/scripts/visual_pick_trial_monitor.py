#!/usr/bin/env python
from __future__ import division

import json
import math
import os
import time

import rospy
import tf
import tf.transformations as transformations
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Log
from std_msgs.msg import String

from brick_visual_pick.pose_metrics import pose_error
from brick_visual_pick.trial_results import PickProgress, is_lifted


class VisualPickTrialMonitor(object):
    def __init__(self):
        self.scenario_id = rospy.get_param('~scenario_id')
        self.result_file = rospy.get_param('~result_file')
        self.expect_pose = rospy.get_param('~expect_pose', True)
        self.timeout = float(rospy.get_param('~timeout', 90.0))
        self.minimum_lift = float(rospy.get_param('~minimum_lift', 0.10))
        self.target_frame = rospy.get_param(
            '~target_frame', 'aubo_i5_base_link')
        self.started_at = time.time()
        self.progress = PickProgress()
        self.status = 'STARTING'
        self.official_pose = None
        self.pose_metrics = None
        self.initial_height = None
        self.final_height = None
        self.terminal = False
        self.terminal_reason = ''
        self.tf_listener = tf.TransformListener()
        self.get_model_state = rospy.ServiceProxy(
            '/gazebo/get_model_state', GetModelState)

        rospy.Subscriber('/visual_pick/status', String, self.status_callback,
                         queue_size=10)
        rospy.Subscriber('/brick_pose', PoseStamped, self.pose_callback,
                         queue_size=1)
        rospy.Subscriber('/rosout_agg', Log, self.log_callback, queue_size=100)

    def brick_height(self):
        try:
            state = self.get_model_state('brick', 'world')
            if state.success:
                return state.pose.position.z
        except rospy.ServiceException:
            pass
        return None

    def status_callback(self, message):
        self.status = message.data
        self.progress.handle_status(message.data)
        if message.data.startswith('PERCEPTION_REJECTED:'):
            self.terminal = True
            self.terminal_reason = message.data

    def pose_callback(self, message):
        if self.official_pose is not None:
            return
        self.official_pose = message
        try:
            state = self.get_model_state('brick', 'world')
            ground_truth = PoseStamped()
            ground_truth.header.frame_id = 'world'
            ground_truth.header.stamp = rospy.Time(0)
            ground_truth.pose = state.pose
            transformed = self.tf_listener.transformPose(
                self.target_frame, ground_truth)
            self.pose_metrics = pose_error(message.pose, transformed.pose)
        except (rospy.ServiceException, tf.Exception):
            self.pose_metrics = None

    def log_callback(self, message):
        if message.name != '/brick_pick_node':
            return
        self.progress.handle_log(message.msg)
        if '[SUCCESS] brick pick and lift completed' in message.msg:
            self.terminal = True
            self.terminal_reason = 'manipulation_completed'
        elif '[ERROR]' in message.msg:
            self.terminal = True
            self.terminal_reason = message.msg

    def result(self):
        self.final_height = self.brick_height()
        lifted = (self.initial_height is not None and
                  self.final_height is not None and
                  is_lifted(self.initial_height, self.final_height,
                            self.minimum_lift))
        end_to_end = self.progress.manipulation_success and lifted
        if self.initial_height is None:
            self.progress.failure_type = 'harness_failed'
            self.progress.failure_detail = 'initial_brick_state_unavailable'
        elif not self.progress.failure_type and not end_to_end and self.expect_pose:
            if not self.progress.perception_success:
                self.progress.failure_type = 'perception_failed'
            elif not self.progress.planning_success:
                self.progress.failure_type = 'planning_failed'
            elif not self.progress.grasp_success:
                self.progress.failure_type = 'grasp_failed'
            else:
                self.progress.failure_type = 'lift_failed'
            self.progress.failure_detail = self.terminal_reason
        control_safe = (not self.expect_pose and
                        self.official_pose is None and
                        not self.progress.execution_started and
                        self.status.startswith('PERCEPTION_REJECTED:'))
        result = {
            'scenario_id': self.scenario_id,
            'expect_pose': bool(self.expect_pose),
            'status': self.status,
            'perception_success': self.progress.perception_success,
            'planning_success': self.progress.planning_success,
            'grasp_success': self.progress.grasp_success,
            'manipulation_success': self.progress.manipulation_success,
            'physical_lift_success': lifted,
            'end_to_end_success': end_to_end,
            'execution_started': self.progress.execution_started,
            'control_safe': control_safe,
            'failure_type': self.progress.failure_type,
            'failure_detail': self.progress.failure_detail,
            'initial_brick_z': self.initial_height,
            'final_brick_z': self.final_height,
            'wall_duration': time.time() - self.started_at,
            'official_pose_frame': (self.official_pose.header.frame_id
                                    if self.official_pose else ''),
            'pose_error': self.pose_metrics,
        }
        return result

    def write_result(self, result):
        directory = os.path.dirname(self.result_file)
        if directory and not os.path.isdir(directory):
            os.makedirs(directory)
        temporary = self.result_file + '.tmp'
        with open(temporary, 'w') as stream:
            json.dump(result, stream, indent=2, sort_keys=True)
            stream.write('\n')
        os.rename(temporary, self.result_file)
        rospy.loginfo('[VISUAL_PICK_TRIAL] %s', json.dumps(result,
                                                          sort_keys=True))

    def run(self):
        try:
            rospy.wait_for_service('/gazebo/get_model_state', timeout=15.0)
            deadline = time.time() + 5.0
            while not rospy.is_shutdown() and time.time() < deadline:
                self.initial_height = self.brick_height()
                if self.initial_height is not None:
                    break
                rospy.rostime.wallsleep(0.1)
            while not rospy.is_shutdown():
                if self.terminal:
                    rospy.rostime.wallsleep(0.8)
                    self.write_result(self.result())
                    return 0
                if time.time() - self.started_at > self.timeout:
                    self.terminal_reason = 'trial_wall_timeout'
                    self.write_result(self.result())
                    return 2
                rospy.rostime.wallsleep(0.1)
        except rospy.ROSException as error:
            self.terminal_reason = 'gazebo_service_timeout:' + str(error)
            self.write_result(self.result())
            return 3


def main():
    rospy.init_node('visual_pick_trial_monitor')
    return VisualPickTrialMonitor().run()


if __name__ == '__main__':
    raise SystemExit(main())
