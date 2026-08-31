#!/usr/bin/env python
from __future__ import division

import math

import message_filters
import numpy as np
import rospy
import tf.transformations as transformations
import tf2_ros
from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import CameraInfo, PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray

from brick_rgbd_perception.brick_geometry import (
    min_cloud_points_for_geometry, oriented_brick_geometry)
from brick_rgbd_perception.pose_estimation import estimate_box_4dof


class BrickPoseEstimator(object):
    def __init__(self):
        self.target_frame = rospy.get_param('~target_frame',
                                             'aubo_i5_base_link')
        self.length = rospy.get_param('~brick_length', 0.240)
        self.width = rospy.get_param('~brick_width', 0.115)
        self.height = rospy.get_param('~brick_height', 0.053)
        self.orientation_mode = rospy.get_param(
            '~brick_orientation_mode', 'flat')
        self.geometry = oriented_brick_geometry(
            self.length, self.width, self.height, self.orientation_mode)
        self.top_tolerance = rospy.get_param('~top_surface_tolerance', 0.008)
        self.min_top_points = rospy.get_param('~min_top_points', 100)
        reference_min_cloud_points = rospy.get_param(
            '~min_cloud_points', 60000)
        reference_top_area = rospy.get_param(
            '~min_cloud_points_reference_top_area', self.length * self.width)
        self.min_cloud_points = min_cloud_points_for_geometry(
            reference_min_cloud_points, reference_top_area, self.geometry)
        self.min_pca_ratio = rospy.get_param('~min_pca_ratio', 1.5)
        self.min_dimension_coverage = rospy.get_param(
            '~min_dimension_coverage', 0.5)
        self.max_dimension_ratio = rospy.get_param('~max_dimension_ratio', 1.25)
        self.tf_timeout = rospy.get_param('~tf_timeout', 0.10)
        self.image_border_margin = rospy.get_param('~image_border_margin', 2.0)
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pose_pub = rospy.Publisher('~pose_debug', PoseStamped,
                                        queue_size=1)
        self.marker_pub = rospy.Publisher('~markers', MarkerArray,
                                          queue_size=1)
        cloud_sub = message_filters.Subscriber(
            rospy.get_param('~cloud_topic', '/brick_rgbd_perception/points'),
            PointCloud2)
        info_sub = message_filters.Subscriber(
            rospy.get_param('~camera_info_topic',
                            '/camera/depth/camera_info'),
            CameraInfo)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [cloud_sub, info_sub],
            rospy.get_param('~info_sync_queue_size', 10),
            rospy.get_param('~info_sync_slop', 0.04))
        self.sync.registerCallback(self.callback)
        rospy.loginfo(
            'brick geometry mode=%s nominal=(%.3f,%.3f,%.3f) '
            'top=(%.3f,%.3f) vertical=%.3f min_cloud_points=%d',
            self.orientation_mode, self.length, self.width, self.height,
            self.geometry.top_length, self.geometry.top_width,
            self.geometry.vertical_height, self.min_cloud_points)

    @staticmethod
    def cloud_xyz(message):
        fields = dict((field.name, field) for field in message.fields)
        xyz = [fields.get(name) for name in ('x', 'y', 'z')]
        if (any(field is None for field in xyz) or
                [field.offset for field in xyz] != [0, 4, 8] or
                any(field.datatype != PointField.FLOAT32 or field.count != 1
                    for field in xyz)):
            raise ValueError('expected contiguous x/y/z float32 fields')
        if message.point_step < 12 or message.point_step % 4:
            raise ValueError('invalid PointCloud2 point_step')
        if (message.height != 1 or
                message.row_step != message.point_step * message.width):
            raise ValueError('expected an unorganized cloud without row padding')
        dtype = '>f4' if message.is_bigendian else '<f4'
        values = np.frombuffer(message.data, dtype=dtype)
        columns = message.point_step // 4
        if values.size % columns:
            raise ValueError('PointCloud2 data length is inconsistent')
        return values.reshape((-1, columns))[:, :3].astype(np.float64)

    @staticmethod
    def transform_matrix(transform):
        return np.dot(
            transformations.translation_matrix((
                transform.translation.x, transform.translation.y,
                transform.translation.z)),
            transformations.quaternion_matrix((
                transform.rotation.x, transform.rotation.y,
                transform.rotation.z, transform.rotation.w)))

    def callback(self, cloud, camera_info):
        point_count = cloud.width * cloud.height
        if point_count < self.min_cloud_points:
            rospy.logwarn_throttle(
                2.0, 'brick pose estimate waiting for complete cloud: %d/%d points',
                point_count, self.min_cloud_points)
            return
        if camera_info.header.frame_id != cloud.header.frame_id:
            rospy.logwarn_throttle(
                2.0, 'cloud/CameraInfo frames differ: %s != %s',
                cloud.header.frame_id, camera_info.header.frame_id)
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, cloud.header.frame_id, cloud.header.stamp,
                rospy.Duration(self.tf_timeout)).transform
            source_points = self.cloud_xyz(cloud)
            camera_k = np.asarray(camera_info.K).reshape((3, 3))
            image_points = np.column_stack((
                camera_k[0, 0] * source_points[:, 0] / source_points[:, 2] +
                camera_k[0, 2],
                camera_k[1, 1] * source_points[:, 1] / source_points[:, 2] +
                camera_k[1, 2]))
            homogeneous = np.column_stack((
                source_points, np.ones(source_points.shape[0])))
            target_points = np.dot(
                self.transform_matrix(transform), homogeneous.T).T[:, :3]
            estimate = estimate_box_4dof(
                target_points, self.geometry.top_length,
                self.geometry.top_width, self.geometry.vertical_height,
                self.top_tolerance, self.min_top_points,
                image_points=image_points,
                image_size=(camera_info.width, camera_info.height),
                image_border_margin=self.image_border_margin,
                min_pca_ratio=self.min_pca_ratio,
                min_dimension_coverage=self.min_dimension_coverage,
                max_dimension_ratio=self.max_dimension_ratio)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException, ValueError) as error:
            rospy.logwarn_throttle(2.0, 'brick pose estimate skipped: %s', error)
            return

        pose = PoseStamped()
        pose.header.stamp = cloud.header.stamp
        pose.header.frame_id = self.target_frame
        pose.pose.position.x = estimate[0]
        pose.pose.position.y = estimate[1]
        pose.pose.position.z = estimate[2]
        quaternion = transformations.quaternion_from_euler(0.0, 0.0,
                                                            estimate[3])
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = quaternion
        self.pose_pub.publish(pose)
        self.marker_pub.publish(self.make_markers(pose, estimate[3]))
        rospy.loginfo_throttle(
            2.0, 'brick 4DoF in %s: xyz=(%.4f, %.4f, %.4f) yaw=%.4f rad',
            self.target_frame, estimate[0], estimate[1], estimate[2],
            estimate[3])

    def make_markers(self, pose, yaw):
        box = Marker()
        box.header = pose.header
        box.ns = 'brick_pose_estimate'
        box.id = 0
        box.type = Marker.CUBE
        box.action = Marker.ADD
        box.pose = pose.pose
        box.scale.x = self.geometry.top_length
        box.scale.y = self.geometry.top_width
        box.scale.z = self.geometry.vertical_height
        box.color.r = 0.1
        box.color.g = 1.0
        box.color.b = 0.1
        box.color.a = 0.28

        arrow = Marker()
        arrow.header = pose.header
        arrow.ns = 'brick_pose_estimate'
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.orientation.w = 1.0
        start = Point(pose.pose.position.x, pose.pose.position.y,
                      pose.pose.position.z +
                      self.geometry.vertical_height / 2.0 + 0.01)
        end = Point(start.x + 0.65 * self.geometry.top_length * math.cos(yaw),
                    start.y + 0.65 * self.geometry.top_length * math.sin(yaw),
                    start.z)
        arrow.points = [start, end]
        arrow.scale.x = 0.012
        arrow.scale.y = 0.025
        arrow.scale.z = 0.030
        arrow.color.r = 0.1
        arrow.color.g = 1.0
        arrow.color.b = 0.1
        arrow.color.a = 1.0
        return MarkerArray(markers=[box, arrow])


if __name__ == '__main__':
    rospy.init_node('brick_pose_estimator')
    BrickPoseEstimator()
    rospy.spin()
