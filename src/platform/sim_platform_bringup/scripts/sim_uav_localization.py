#!/usr/bin/env python3
"""Publish the SIM map/odom correction from same-time physical and estimated poses.

Gazebo body truth stays in a local buffer. Prometheus remains the sole public
odom/base authority; this adapter publishes only map -> uav1/odom.
"""

import numpy as np


def rigid_matrix(value):
    """Validate a finite, proper homogeneous rigid transform."""
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError("expected a finite 4x4 rigid transform")
    rotation = matrix[:3, :3]
    if (not np.allclose(matrix[3], [0, 0, 0, 1], rtol=0, atol=1e-9)
            or not np.allclose(rotation.T @ rotation, np.eye(3), rtol=0, atol=1e-6)
            or not np.isclose(np.linalg.det(rotation), 1.0, rtol=0, atol=1e-6)):
        raise ValueError("expected a proper rigid rotation and homogeneous bottom row")
    return matrix


def map_to_odom(T_world_base, T_odom_base, T_map_world):
    """Recompose physical map/base without changing the estimator's odom/base."""
    world_base = rigid_matrix(T_world_base)
    odom_base = rigid_matrix(T_odom_base)
    map_world = rigid_matrix(T_map_world)
    return rigid_matrix(map_world @ world_base @ np.linalg.inv(odom_base))


def pose_matrix(translation, quaternion):
    """Convert finite position and a unit ROS xyzw quaternion to a rigid matrix."""
    position = np.asarray(translation, dtype=float)
    orientation = np.asarray(quaternion, dtype=float)
    if (position.shape != (3,) or orientation.shape != (4,)
            or not np.isfinite(position).all() or not np.isfinite(orientation).all()
            or not np.isclose(np.linalg.norm(orientation), 1.0, rtol=0, atol=1e-6)):
        raise ValueError("expected finite xyz position and unit xyzw quaternion")
    x, y, z, w = orientation / np.linalg.norm(orientation)
    matrix = np.eye(4)
    matrix[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    matrix[:3, 3] = position
    return matrix


def rotation_quaternion(matrix):
    """Return a unit xyzw quaternion, including rotations near a half turn."""
    rotation = rigid_matrix(matrix)[:3, :3]
    xx, xy, xz = rotation[0]
    yx, yy, yz = rotation[1]
    zx, zy, zz = rotation[2]
    # Symmetric eigenproblem avoids division by a near-zero scalar component.
    symmetric = np.array([
        [xx - yy - zz, xy + yx, xz + zx, zy - yz],
        [xy + yx, yy - xx - zz, yz + zy, xz - zx],
        [xz + zx, yz + zy, zz - xx - yy, yx - xy],
        [zy - yz, xz - zx, yx - xy, xx + yy + zz],
    ])
    _, eigenvectors = np.linalg.eigh(symmetric)
    quaternion = eigenvectors[:, -1]
    return -quaternion if quaternion[3] < 0 else quaternion


def fresh_stamp(stamp_ns, now_ns, previous_ns):
    """Accept a new positive stamp no more than 0.5 seconds behind the clock."""
    return (stamp_ns > 0 and stamp_ns > previous_ns
            and 0 <= now_ns - stamp_ns <= 500000000)


def transform_matrix(message):
    """Convert a TransformStamped without requiring ROS imports in pure tests."""
    translation = message.transform.translation
    rotation = message.transform.rotation
    return pose_matrix((translation.x, translation.y, translation.z),
                       (rotation.x, rotation.y, rotation.z, rotation.w))


def localization_at_latest_odom(public_buffer, truth_buffer, now, previous_ns):
    """Return (stamp, map/odom) only when truth is available at that odom stamp.

    All lookups use the default zero timeout. Missing data raises the buffer's
    lookup exception so the caller can retry on the next timer tick.
    """
    estimated = public_buffer.lookup_transform(
        "uav1/odom", "uav1/base_link", type(now)())
    stamp = estimated.header.stamp
    if not fresh_stamp(stamp.to_nsec(), now.to_nsec(), previous_ns):
        return None
    physical = truth_buffer.lookup_transform("world", "sim_uav1_body", stamp)
    map_world = public_buffer.lookup_transform("map", "world", stamp)
    correction = map_to_odom(transform_matrix(physical), transform_matrix(estimated),
                            transform_matrix(map_world))
    return stamp, correction


class SimUavLocalization:
    """ROS wiring for the bounded SIM localization adapter."""

    def __init__(self):
        import rospy
        import tf2_ros
        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import Odometry

        self.ros = rospy
        self.transform_type = TransformStamped
        self.lookup_errors = (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                              tf2_ros.ExtrapolationException, ValueError)
        self.public_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0), debug=False)
        self.truth_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0), debug=False)
        self.listener = tf2_ros.TransformListener(self.public_buffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.previous_ns = 0
        self.subscriber = rospy.Subscriber("/sim/uav1/base_pose", Odometry,
                                          self.receive_truth, queue_size=100)
        self.timer = rospy.Timer(rospy.Duration(0.02), self.publish_correction)

    def receive_truth(self, message):
        """Cache exact stamped world/base_link truth under a private child name."""
        if (message.header.frame_id != "world" or message.child_frame_id != "base_link"
                or message.header.stamp.to_nsec() <= 0):
            self.ros.logwarn_throttle(
                5.0, "SIM body pose requires world/base_link and a positive stamp")
            return
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        try:
            pose_matrix((position.x, position.y, position.z),
                        (orientation.x, orientation.y, orientation.z, orientation.w))
        except ValueError as error:
            self.ros.logwarn_throttle(5.0, "Invalid SIM body pose: %s", error)
            return
        transform = self.transform_type()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = "world"
        transform.child_frame_id = "sim_uav1_body"
        transform.transform.translation.x = position.x
        transform.transform.translation.y = position.y
        transform.transform.translation.z = position.z
        transform.transform.rotation = orientation
        self.truth_buffer.set_transform(transform, "sim_base_pose")

    def publish_correction(self, _event):
        try:
            sample = localization_at_latest_odom(
                self.public_buffer, self.truth_buffer, self.ros.Time.now(), self.previous_ns)
            if sample is None:
                return
            stamp, correction = sample
            quaternion = rotation_quaternion(correction)
        except self.lookup_errors as error:
            self.ros.logdebug_throttle(5.0, "Waiting for same-time SIM localization: %s", error)
            return
        transform = self.transform_type()
        transform.header.stamp = stamp
        transform.header.frame_id = "map"
        transform.child_frame_id = "uav1/odom"
        (transform.transform.translation.x, transform.transform.translation.y,
         transform.transform.translation.z) = correction[:3, 3]
        (transform.transform.rotation.x, transform.transform.rotation.y,
         transform.transform.rotation.z, transform.transform.rotation.w) = quaternion
        self.broadcaster.sendTransform(transform)
        self.previous_ns = stamp.to_nsec()


def main():
    import rospy

    rospy.init_node("sim_localization_uav1")
    adapter = SimUavLocalization()
    rospy.spin()


if __name__ == "__main__":
    main()
