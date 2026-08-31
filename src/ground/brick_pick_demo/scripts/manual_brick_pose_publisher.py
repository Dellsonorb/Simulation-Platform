#!/usr/bin/env python
from __future__ import print_function

import math
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler


def main():
    rospy.init_node('manual_brick_pose_publisher')
    publisher = rospy.Publisher('/brick_pose', PoseStamped, queue_size=1, latch=True)
    pose = PoseStamped()
    pose.header.frame_id = rospy.get_param('~frame_id', 'world')
    pose.pose.position.x = rospy.get_param('~x', 0.75)
    pose.pose.position.y = rospy.get_param('~y', 0.0)
    pose.pose.position.z = rospy.get_param('~z', 0.0265)
    yaw = rospy.get_param('~yaw', 0.0)
    quaternion = quaternion_from_euler(0.0, 0.0, yaw)
    pose.pose.orientation.x, pose.pose.orientation.y = quaternion[0:2]
    pose.pose.orientation.z, pose.pose.orientation.w = quaternion[2:4]
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        pose.header.stamp = rospy.Time.now()
        publisher.publish(pose)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
