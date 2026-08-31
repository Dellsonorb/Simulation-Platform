#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import csv
import datetime
import json
import math
import os
import sys
import subprocess
import threading
import time

import numpy as np
import rospy
import tf.transformations as transformations
import tf2_ros
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2

from brick_rgbd_perception.validation import (
    observation_joint_error, pose_errors, representative_pose,
    select_stamped_samples, summarize_scenarios, validate_scenarios)


class PoseRobustnessValidator(object):
    """Move the Gazebo brick and measure the unchanged estimator."""

    def __init__(self):
        self.scenarios = rospy.get_param('~scenarios')
        validate_scenarios(self.scenarios)
        self.model_name = rospy.get_param('~model_name', 'brick')
        self.target_frame = rospy.get_param(
            '~target_frame', 'aubo_i5_base_link')
        self.brick_z = float(rospy.get_param('~brick_z', 0.0265))
        self.sample_count = int(rospy.get_param('~sample_count', 7))
        self.minimum_sample_span = float(rospy.get_param(
            '~minimum_sample_span', 0.30))
        self.settle_duration = float(rospy.get_param(
            '~settle_duration', 0.8))
        self.sample_timeout = float(rospy.get_param('~sample_timeout', 3.0))
        self.position_stability = float(rospy.get_param(
            '~position_stability', 0.003))
        self.yaw_stability = float(rospy.get_param(
            '~yaw_stability', 0.02))
        self.min_cloud_points = int(rospy.get_param(
            '~min_cloud_points', 60000))
        self.border_margin = int(rospy.get_param('~border_margin', 2))
        joint_names = rospy.get_param('~observation/joint_names')
        joint_positions = rospy.get_param('~observation/positions')
        if len(joint_names) != len(joint_positions):
            raise ValueError('observation joint names/positions differ')
        self.expected_joints = dict(zip(joint_names, joint_positions))
        self.observation_tolerance = float(rospy.get_param(
            '~observation/goal_tolerance', 0.08))
        self.observation_timeout = float(rospy.get_param(
            '~observation_timeout', 30.0))
        self.max_truth_position_drift = float(rospy.get_param(
            '~max_truth_position_drift', 0.0005))
        self.max_truth_yaw_drift = float(rospy.get_param(
            '~max_truth_yaw_drift', 0.005))
        self.output_dir = os.path.abspath(os.path.expanduser(
            rospy.get_param('~output_dir', '/tmp/brick_pose_robustness')))
        self.strict = bool(rospy.get_param('~strict', True))
        self.max_errors = {
            'xy': float(rospy.get_param('~max_xy_error', 0.008)),
            'z': float(rospy.get_param('~max_z_error', 0.003)),
            'position': float(rospy.get_param(
                '~max_position_error', 0.009)),
            'yaw': float(rospy.get_param('~max_yaw_error', 0.035)),
        }

        self.lock = threading.Lock()
        self.active_start = rospy.Time(0)
        self.pose_messages = []
        self.point_counts = {}
        self.mask_observations = {}
        self.best_mask = None
        self.camera_info = None
        self.joint_positions = {}
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(60.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pose_sub = rospy.Subscriber(
            rospy.get_param('~pose_topic',
                            '/brick_pose_estimator/pose_debug'),
            PoseStamped, self.pose_callback, queue_size=50)
        self.cloud_sub = rospy.Subscriber(
            rospy.get_param('~cloud_topic',
                            '/brick_rgbd_perception/points'),
            PointCloud2, self.cloud_callback, queue_size=5)
        self.mask_sub = rospy.Subscriber(
            rospy.get_param('~mask_topic',
                            '/brick_rgbd_perception/mask'),
            Image, self.mask_callback, queue_size=5)
        self.info_sub = rospy.Subscriber(
            rospy.get_param('~camera_info_topic',
                            '/camera/depth/camera_info'),
            CameraInfo, self.info_callback, queue_size=1)
        self.joint_sub = rospy.Subscriber(
            '/joint_states', JointState, self.joint_callback, queue_size=5)

        rospy.wait_for_service('/gazebo/set_model_state', timeout=30.0)
        rospy.wait_for_service('/gazebo/get_model_state', timeout=30.0)
        self.set_model = rospy.ServiceProxy(
            '/gazebo/set_model_state', SetModelState)
        self.get_model = rospy.ServiceProxy(
            '/gazebo/get_model_state', GetModelState)

    def pose_callback(self, message):
        with self.lock:
            stamp = message.header.stamp.to_nsec()
            if (message.header.stamp >= self.active_start and
                    all(existing.header.stamp.to_nsec() != stamp
                        for existing in self.pose_messages)):
                self.pose_messages.append(message)

    def cloud_callback(self, message):
        with self.lock:
            if message.header.stamp >= self.active_start:
                self.point_counts[message.header.stamp.to_nsec()] = (
                    message.width * message.height)

    def info_callback(self, message):
        with self.lock:
            self.camera_info = message

    def joint_callback(self, message):
        with self.lock:
            self.joint_positions.update(dict(zip(message.name,
                                                 message.position)))

    def mask_callback(self, message):
        if message.encoding not in ('mono8', '8UC1'):
            return
        with self.lock:
            if message.header.stamp < self.active_start:
                return
        values = np.frombuffer(message.data, dtype=np.uint8)
        if values.size != message.height * message.step:
            return
        mask = values.reshape((message.height, message.step))[:, :message.width]
        rows, columns = np.nonzero(mask)
        if not rows.size:
            observation = {'pixels': 0, 'border': False, 'bbox': None}
        else:
            margin = self.border_margin
            observation = {
                'pixels': int(rows.size),
                'border': bool(
                    rows.min() <= margin or columns.min() <= margin or
                    rows.max() >= message.height - 1 - margin or
                    columns.max() >= message.width - 1 - margin),
                'bbox': [int(columns.min()), int(rows.min()),
                         int(columns.max()), int(rows.max())],
            }
        with self.lock:
            if message.header.stamp >= self.active_start:
                self.mask_observations[
                    message.header.stamp.to_nsec()] = observation
            if (message.header.stamp >= self.active_start and
                    (self.best_mask is None or
                     observation['pixels'] > self.best_mask['pixels'])):
                self.best_mask = observation

    @staticmethod
    def transform_matrix(transform):
        return np.dot(
            transformations.translation_matrix((
                transform.translation.x, transform.translation.y,
                transform.translation.z)),
            transformations.quaternion_matrix((
                transform.rotation.x, transform.rotation.y,
                transform.rotation.z, transform.rotation.w)))

    def place_brick(self, scenario):
        state = ModelState()
        state.model_name = self.model_name
        state.reference_frame = 'world'
        state.pose.position.x = float(scenario['x'])
        state.pose.position.y = float(scenario['y'])
        state.pose.position.z = self.brick_z
        quaternion = transformations.quaternion_from_euler(
            0.0, 0.0, float(scenario['yaw']))
        (state.pose.orientation.x, state.pose.orientation.y,
         state.pose.orientation.z, state.pose.orientation.w) = quaternion
        for unused_attempt in range(3):
            response = self.set_model(state)
            if not response.success:
                raise RuntimeError('failed to place brick: %s' %
                                   response.status_message)
            rospy.sleep(0.10)
        rospy.sleep(self.settle_duration)

    def reset_observations(self):
        with self.lock:
            self.active_start = rospy.Time.now()
            self.pose_messages = []
            self.point_counts = {}
            self.mask_observations = {}
            self.best_mask = None

    def wait_for_samples(self):
        deadline = time.time() + self.sample_timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            with self.lock:
                stamped = [(message.header.stamp.to_sec(), message)
                           for message in self.pose_messages]
                selected = select_stamped_samples(
                    stamped, self.sample_count, self.minimum_sample_span)
                if selected is not None:
                    break
            rospy.sleep(0.03)
        with self.lock:
            stamped = [(message.header.stamp.to_sec(), message)
                       for message in self.pose_messages]
            selected = select_stamped_samples(
                stamped, self.sample_count, self.minimum_sample_span)
            messages = ([item[1] for item in selected]
                        if selected is not None else [])
            return (messages, dict(self.point_counts),
                    dict(self.mask_observations),
                    dict(self.best_mask) if self.best_mask else None)

    def world_pose(self):
        response = self.get_model(self.model_name, 'world')
        if not response.success:
            raise RuntimeError('failed to read brick ground truth: %s' %
                               response.status_message)
        return response.pose

    def ground_truth(self, world_pose, stamp):
        transform = self.tf_buffer.lookup_transform(
            self.target_frame, 'world', stamp,
            rospy.Duration(5.0)).transform
        target_from_world = self.transform_matrix(transform)
        world_from_brick = np.dot(
            transformations.translation_matrix((
                world_pose.position.x, world_pose.position.y,
                world_pose.position.z)),
            transformations.quaternion_matrix((
                world_pose.orientation.x, world_pose.orientation.y,
                world_pose.orientation.z, world_pose.orientation.w)))
        target_from_brick = np.dot(target_from_world, world_from_brick)
        position = target_from_brick[:3, 3]
        yaw = math.atan2(target_from_brick[1, 0], target_from_brick[0, 0])
        return position, yaw

    def project_truth_center(self, world_pose, stamp):
        with self.lock:
            camera_info = self.camera_info
        if camera_info is None:
            return None
        transform = self.tf_buffer.lookup_transform(
            camera_info.header.frame_id, 'world', stamp,
            rospy.Duration(5.0)).transform
        point = np.array((world_pose.position.x, world_pose.position.y,
                          world_pose.position.z, 1.0))
        camera_point = np.dot(self.transform_matrix(transform), point)
        if camera_point[2] <= 0.0:
            return None
        k = np.asarray(camera_info.K).reshape((3, 3))
        return [float(k[0, 0] * camera_point[0] / camera_point[2] + k[0, 2]),
                float(k[1, 1] * camera_point[1] / camera_point[2] + k[1, 2])]

    @staticmethod
    def message_pose(message):
        quaternion = (
            message.pose.orientation.x, message.pose.orientation.y,
            message.pose.orientation.z, message.pose.orientation.w)
        rotation = transformations.quaternion_matrix(quaternion)
        return ((message.pose.position.x, message.pose.position.y,
                 message.pose.position.z),
                math.atan2(rotation[1, 0], rotation[0, 0]))

    @staticmethod
    def world_pose_4dof(world_pose):
        rotation = transformations.quaternion_matrix((
            world_pose.orientation.x, world_pose.orientation.y,
            world_pose.orientation.z, world_pose.orientation.w))
        return ((world_pose.position.x, world_pose.position.y,
                 world_pose.position.z),
                math.atan2(rotation[1, 0], rotation[0, 0]))

    def wait_for_observation_pose(self):
        deadline = time.time() + self.observation_timeout
        stable_since = None
        while not rospy.is_shutdown() and time.time() < deadline:
            with self.lock:
                actual = dict(self.joint_positions)
            try:
                error = observation_joint_error(actual,
                                                self.expected_joints)
            except ValueError:
                error = float('inf')
            if error <= self.observation_tolerance:
                if stable_since is None:
                    stable_since = time.time()
                if time.time() - stable_since >= 0.5:
                    rospy.loginfo('observation pose verified; maximum joint '
                                  'error %.4f rad', error)
                    return
            else:
                stable_since = None
            rospy.sleep(0.05)
        raise RuntimeError('arm did not reach the configured observation pose')

    def run_scenario(self, index, scenario):
        self.place_brick(scenario)
        truth_before = self.world_pose()
        self.reset_observations()
        messages, point_counts, masks, best_mask = self.wait_for_samples()
        truth_after = self.world_pose()
        before_position, before_yaw = self.world_pose_4dof(truth_before)
        after_position, after_yaw = self.world_pose_4dof(truth_after)
        truth_drift = pose_errors(after_position, after_yaw,
                                  before_position, before_yaw)
        representative_stamp = (messages[len(messages) // 2].header.stamp
                                if messages else rospy.Time.now())
        truth_position, truth_yaw = self.ground_truth(
            truth_after, representative_stamp)
        matched_masks = [masks.get(message.header.stamp.to_nsec())
                         for message in messages]
        available_matched_masks = [mask for mask in matched_masks
                                   if mask is not None]
        mask = (max(available_matched_masks,
                    key=lambda item: item['pixels'])
                if available_matched_masks else
                (best_mask if not messages else None))
        matched_counts = [point_counts.get(message.header.stamp.to_nsec(), 0)
                          for message in messages]
        result = dict(scenario)
        result.update({
            'index': index,
            'sample_count': len(messages),
            'max_points': (max(point_counts.values())
                           if point_counts else 0),
            'mask_pixels': mask['pixels'] if mask else 0,
            'border_touched': mask['border'] if mask else False,
            'mask_bbox': mask['bbox'] if mask else None,
            'projected_center': self.project_truth_center(
                truth_after, representative_stamp),
            'truth_position': [float(value) for value in truth_position],
            'truth_yaw': float(truth_yaw),
            'truth_drift': truth_drift,
            'representative_stamp': representative_stamp.to_sec(),
            'errors': None,
            'spread': None,
            'samples': [],
        })
        if len(messages) < self.sample_count:
            result['outcome'] = 'rejected'
            if not result['max_points']:
                result['reason'] = 'no_segmented_cloud'
            elif result['max_points'] < self.min_cloud_points:
                result['reason'] = 'cloud_below_min_points'
            else:
                result['reason'] = 'geometry_or_partial_extent_rejected'
            return result

        samples = [self.message_pose(message) for message in messages]
        result['samples'] = [{
            'stamp': message.header.stamp.to_sec(),
            'position': [float(value) for value in sample[0]],
            'yaw': float(sample[1]),
            'points': matched_counts[sample_index],
            'mask': masks.get(message.header.stamp.to_nsec()),
        } for sample_index, (message, sample) in enumerate(
            zip(messages, samples))]
        estimate_position, estimate_yaw, spread = representative_pose(samples)
        result['spread'] = spread
        result['estimate_position'] = [float(value)
                                       for value in estimate_position]
        result['estimate_yaw'] = float(estimate_yaw)
        result['errors'] = pose_errors(
            estimate_position, estimate_yaw, truth_position, truth_yaw)
        if (truth_drift['position'] > self.max_truth_position_drift or
                truth_drift['yaw'] > self.max_truth_yaw_drift):
            result['outcome'] = 'rejected'
            result['reason'] = 'ground_truth_moved_during_sampling'
            result['errors'] = None
        elif (spread['position_max'] > self.position_stability or
                spread['yaw_max'] > self.yaw_stability):
            result['outcome'] = 'rejected'
            result['reason'] = 'unstable_multiframe_estimate'
            result['errors'] = None
        else:
            result['outcome'] = 'success'
            result['reason'] = ('boundary_visible_success' if
                                result['border_touched'] else 'complete_view')
        return result

    def run(self):
        while not rospy.is_shutdown():
            with self.lock:
                ready = self.camera_info is not None
            if ready:
                break
            rospy.sleep(0.05)
        self.wait_for_observation_pose()
        results = []
        for index, scenario in enumerate(self.scenarios):
            rospy.loginfo('validation pose %02d/%02d %s: x=%.3f y=%.3f '
                          'yaw=%.1f deg', index + 1, len(self.scenarios),
                          scenario['id'], scenario['x'], scenario['y'],
                          math.degrees(scenario['yaw']))
            result = self.run_scenario(index, scenario)
            results.append(result)
            rospy.loginfo('  outcome=%s reason=%s samples=%d points=%d '
                          'border=%s', result['outcome'], result['reason'],
                          result['sample_count'], result['max_points'],
                          result['border_touched'])
        summary = summarize_scenarios(results)
        self.write_results(results, summary)
        return self.check_results(results, summary)

    def check_results(self, results, summary):
        failures = []
        for result in results:
            if result['outcome'] != result['expect']:
                failures.append('%s expected %s, got %s' % (
                    result['id'], result['expect'], result['outcome']))
            if result['outcome'] == 'success':
                if any(sample['mask'] is None
                       for sample in result['samples']):
                    failures.append('%s has an unpaired mask frame' %
                                    result['id'])
                for name, limit in self.max_errors.items():
                    if result['errors'][name] > limit:
                        failures.append('%s %s %.6f > %.6f' % (
                            result['id'], name, result['errors'][name], limit))
        compensated = [result for result in results
                       if result['outcome'] == 'success' and
                       result['border_touched']]
        evidenced_rejections = [result for result in results
                               if result['outcome'] == 'rejected' and
                               result['reason'] in (
                                   'cloud_below_min_points',
                                   'geometry_or_partial_extent_rejected')]
        if not compensated:
            failures.append('no successful image-boundary compensation case')
        if not evidenced_rejections:
            failures.append('no estimator rejection case was observed')
        if '/brick_pose' in dict(rospy.get_published_topics()):
            failures.append('formal /brick_pose must remain unpublished')
        rospy.loginfo('multi-pose summary: %d/%d success (%.1f%%), %d/%d '
                      'rejected (%.1f%%)', summary['successes'],
                      summary['total'], 100.0 * summary['success_rate'],
                      summary['rejections'], summary['total'],
                      100.0 * summary['rejection_rate'])
        if failures:
            for failure in failures:
                rospy.logerr('validation failure: %s', failure)
            return not self.strict
        return True

    def write_results(self, results, summary):
        if not os.path.isdir(self.output_dir):
            os.makedirs(self.output_dir)
        json_path = os.path.join(self.output_dir,
                                 'brick_pose_robustness.json')
        csv_path = os.path.join(self.output_dir,
                                'brick_pose_robustness.csv')
        report_path = os.path.join(self.output_dir,
                                   'BRICK_POSE_ROBUSTNESS_REPORT.md')
        try:
            commit = subprocess.check_output(
                ['git', '-c', 'safe.directory=/ws', 'rev-parse', 'HEAD'],
                cwd='/ws').strip()
        except (OSError, subprocess.CalledProcessError):
            commit = 'unknown'
        metadata = {
            'generated_at_utc': datetime.datetime.utcnow().isoformat() + 'Z',
            'ros_run_id': rospy.get_param('/run_id', 'unknown'),
            'git_commit': commit,
            'parameters': {
                'sample_count': self.sample_count,
                'minimum_sample_span': self.minimum_sample_span,
                'min_cloud_points': self.min_cloud_points,
                'position_stability': self.position_stability,
                'yaw_stability': self.yaw_stability,
                'max_errors': self.max_errors,
                'max_truth_position_drift': self.max_truth_position_drift,
                'max_truth_yaw_drift': self.max_truth_yaw_drift,
            },
        }
        with open(json_path, 'w') as stream:
            json.dump({'metadata': metadata, 'summary': summary,
                       'scenarios': results}, stream,
                      indent=2, sort_keys=True, separators=(',', ': '))
            stream.write('\n')
        fieldnames = [
            'index', 'id', 'category', 'x', 'y', 'yaw_deg', 'expect',
            'outcome', 'reason', 'sample_count', 'max_points', 'mask_pixels',
            'border_touched', 'mask_bbox', 'projected_u', 'projected_v',
            'xy_mm', 'z_mm', 'position_mm', 'yaw_deg_error']
        with open(csv_path, 'wb') as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames,
                                    lineterminator='\n')
            writer.writeheader()
            for result in results:
                errors = result.get('errors') or {}
                projected = result.get('projected_center') or (None, None)
                writer.writerow({
                    'index': result['index'] + 1,
                    'id': result['id'],
                    'category': result['category'],
                    'x': result['x'], 'y': result['y'],
                    'yaw_deg': math.degrees(result['yaw']),
                    'expect': result['expect'],
                    'outcome': result['outcome'], 'reason': result['reason'],
                    'sample_count': result['sample_count'],
                    'max_points': result['max_points'],
                    'mask_pixels': result['mask_pixels'],
                    'border_touched': result['border_touched'],
                    'mask_bbox': result['mask_bbox'],
                    'projected_u': projected[0], 'projected_v': projected[1],
                    'xy_mm': (1000.0 * errors['xy'] if errors else None),
                    'z_mm': (1000.0 * errors['z'] if errors else None),
                    'position_mm': (1000.0 * errors['position']
                                    if errors else None),
                    'yaw_deg_error': (math.degrees(errors['yaw'])
                                      if errors else None),
                })
        lines = [
            '# Brick Pose Estimator 多位姿鲁棒性实测', '',
            '- 位姿总数：%d' % summary['total'],
            '- 成功：%d（%.1f%%）' % (
                summary['successes'], 100.0 * summary['success_rate']),
            '- 拒绝：%d（%.1f%%）' % (
                summary['rejections'], 100.0 * summary['rejection_rate']),
            '- 预期结果匹配率：%.1f%%' % (
                100.0 * summary['expected_match_rate']),
            '- 运行时间（UTC）：%s' % metadata['generated_at_utc'],
            '- ROS run ID：`%s`' % metadata['ros_run_id'],
            '- Git commit：`%s`' % metadata['git_commit'],
            '- 误差统计口径：仅统计 %d 个成功场景，每个场景使用 7 个唯一帧的代表位姿。' %
            summary['successes'],
            '- Strict 门限：XY %.1f mm，Z %.1f mm，Position %.1f mm，Yaw %.2f deg。' % (
                1000.0 * self.max_errors['xy'],
                1000.0 * self.max_errors['z'],
                1000.0 * self.max_errors['position'],
                math.degrees(self.max_errors['yaw'])),
            '',
            '| 指标 | Mean | Median | Max |',
            '|---|---:|---:|---:|']
        if summary['successes']:
            for name in ('xy', 'z', 'position'):
                metric = summary['metrics'][name]
                lines.append('| %s | %.3f mm | %.3f mm | %.3f mm |' % (
                    name.upper(), 1000.0 * metric['mean'],
                    1000.0 * metric['median'], 1000.0 * metric['max']))
            yaw_metric = summary['metrics']['yaw']
            lines.append('| Yaw | %.3f deg | %.3f deg | %.3f deg |' % (
                math.degrees(yaw_metric['mean']),
                math.degrees(yaw_metric['median']),
                math.degrees(yaw_metric['max'])))
        else:
            lines.extend(['| XY | — | — | — |', '| Z | — | — | — |',
                          '| POSITION | — | — | — |',
                          '| Yaw | — | — | — |'])
        compensated_count = sum(
            1 for result in results
            if result['outcome'] == 'success' and result['border_touched'])
        rejection_reasons = {}
        for result in results:
            if result['outcome'] == 'rejected':
                rejection_reasons[result['reason']] = (
                    rejection_reasons.get(result['reason'], 0) + 1)
        lines.extend([
            '', '## 边界与拒绝机制证据', '',
            '- 与所采用位姿帧同时间戳的 Mask 触边、且仍获得稳定估计：%d 组。' %
            compensated_count,
            '- 这直接证明边界可见场景的端到端有效性；内部截断补偿分支由既有单元测试单独覆盖。',
            '- 拒绝原因：%s。' % ', '.join(
                '%s=%d' % item for item in sorted(rejection_reasons.items())),
            '- 当前点数门限：%d；拒绝样本最大点数均记录于下表。' %
            self.min_cloud_points])
        lines.extend(['', '## 分层结果', '',
                      '| 类别 | 成功 | 拒绝 | 总数 |',
                      '|---|---:|---:|---:|'])
        for category in sorted(summary['categories']):
            counts = summary['categories'][category]
            lines.append('| %s | %d | %d | %d |' % (
                category, counts['successes'], counts['rejections'],
                counts['total']))
        lines.extend(['', '## 最差成功案例', ''])
        if summary['successes']:
            for name in ('xy', 'z', 'position', 'yaw'):
                metric = summary['metrics'][name]
                worst = results[metric['worst_index']]
                value = (math.degrees(metric['max']) if name == 'yaw'
                         else 1000.0 * metric['max'])
                unit = 'deg' if name == 'yaw' else 'mm'
                lines.append('- %s：`%s`，%.3f %s' % (
                    name.upper(), worst['id'], value, unit))
        else:
            lines.append('- 无成功案例。')
        lines.extend(['', '| # | ID | 类别 | 期望/结果 | 点数 | 触边 | '
                      'XY / Z / Position / Yaw error |',
                      '|---:|---|---|---|---:|:---:|---|'])
        for result in results:
            errors = result.get('errors')
            error_text = ('—' if not errors else
                          '%.2f / %.2f / %.2f mm / %.2f deg' % (
                              1000.0 * errors['xy'], 1000.0 * errors['z'],
                              1000.0 * errors['position'],
                              math.degrees(errors['yaw'])))
            lines.append('| %d | %s | %s | %s/%s | %d | %s | %s |' % (
                result['index'] + 1, result['id'], result['category'],
                result['expect'], result['outcome'], result['max_points'],
                'yes' if result['border_touched'] else 'no', error_text))
        with open(report_path, 'w') as stream:
            stream.write('\n'.join(lines) + '\n')
        rospy.loginfo('validation reports written to %s', self.output_dir)


if __name__ == '__main__':
    rospy.init_node('brick_pose_robustness_validator')
    try:
        success = PoseRobustnessValidator().run()
    except Exception as error:
        rospy.logfatal('pose robustness validation aborted: %s', error)
        success = False
    sys.exit(0 if success else 1)
