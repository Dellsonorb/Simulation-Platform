#!/usr/bin/env python
from __future__ import division

import cv2
import message_filters
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

from brick_rgbd_perception.core import backproject_mask, segment_hsv


class BrickRgbdNode(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.min_depth = rospy.get_param('~min_depth', 0.15)
        self.max_depth = rospy.get_param('~max_depth', 2.0)
        self.pixel_stride = rospy.get_param('~pixel_stride', 1)
        self.hue_ranges = rospy.get_param('~hue_ranges', [[0, 10], [170, 179]])
        self.saturation_min = rospy.get_param('~saturation_min', 100)
        self.value_min = rospy.get_param('~value_min', 40)
        self.morphology_kernel = rospy.get_param('~morphology_kernel', 3)
        self.min_component_pixels = rospy.get_param('~min_component_pixels', 100)
        queue_size = rospy.get_param('~sync_queue_size', 10)
        slop = rospy.get_param('~sync_slop', 0.04)

        self.mask_pub = rospy.Publisher('~mask', Image, queue_size=1)
        self.debug_pub = rospy.Publisher('~debug_rgb', Image, queue_size=1)
        self.cloud_pub = rospy.Publisher('~points', PointCloud2, queue_size=1)

        color = message_filters.Subscriber(
            rospy.get_param('~color_topic', '/camera/color/image_raw'), Image)
        depth = message_filters.Subscriber(
            rospy.get_param('~depth_topic', '/camera/depth/image_rect_raw'), Image)
        color_info = message_filters.Subscriber(
            rospy.get_param('~color_info_topic', '/camera/color/camera_info'),
            CameraInfo)
        depth_info = message_filters.Subscriber(
            rospy.get_param('~depth_info_topic', '/camera/depth/camera_info'),
            CameraInfo)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [color, depth, color_info, depth_info], queue_size, slop)
        self.sync.registerCallback(self.callback)
        rospy.loginfo('brick RGB-D synchronizer ready: queue=%d slop=%.3f s',
                      queue_size, slop)

    def callback(self, color_msg, depth_msg, color_info, depth_info):
        if depth_msg.encoding != '32FC1':
            rospy.logwarn_throttle(5.0, 'expected 32FC1 depth, got %s',
                                   depth_msg.encoding)
            return
        if (color_msg.width != depth_msg.width or
                color_msg.height != depth_msg.height):
            rospy.logwarn_throttle(5.0, 'RGB/depth dimensions differ; dropping frame')
            return
        color_k = np.asarray(color_info.K).reshape((3, 3))
        depth_k = np.asarray(depth_info.K).reshape((3, 3))
        if not np.allclose(color_k, depth_k, rtol=0.0, atol=1e-6):
            rospy.logwarn_throttle(5.0, 'RGB/depth K differ; aligned depth is required')
            return
        try:
            rgb = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='rgb8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg,
                                              desired_encoding='32FC1')
        except CvBridgeError as error:
            rospy.logerr_throttle(5.0, 'cv_bridge conversion failed: %s', error)
            return

        mask = segment_hsv(
            rgb, self.hue_ranges, self.saturation_min, self.value_min,
            self.morphology_kernel, self.min_component_pixels)
        points, _ = backproject_mask(
            depth, mask, depth_k, self.min_depth, self.max_depth,
            self.pixel_stride)

        mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        mask_msg.header = color_msg.header
        self.mask_pub.publish(mask_msg)

        debug_rgb = rgb.copy()
        contour_result = cv2.findContours(
            mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contour_result[-2]
        cv2.drawContours(debug_rgb, contours, -1, (0, 255, 0), 2)
        debug_msg = self.bridge.cv2_to_imgmsg(debug_rgb, encoding='rgb8')
        debug_msg.header = color_msg.header
        self.debug_pub.publish(debug_msg)

        cloud = self.make_cloud(points, depth_msg.header)
        self.cloud_pub.publish(cloud)
        if points.shape[0]:
            rospy.loginfo_throttle(
                2.0, 'brick points=%d depth=[%.3f, %.3f] m sync_dt=%.3f s',
                points.shape[0], points[:, 2].min(), points[:, 2].max(),
                abs((color_msg.header.stamp - depth_msg.header.stamp).to_sec()))
        else:
            rospy.logwarn_throttle(2.0, 'brick mask has no valid depth points')

    @staticmethod
    def make_cloud(points, header):
        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = points.shape[0]
        cloud.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        cloud.data = np.asarray(points, dtype='<f4').tostring()
        return cloud


if __name__ == '__main__':
    rospy.init_node('brick_rgbd_perception')
    BrickRgbdNode()
    rospy.spin()
