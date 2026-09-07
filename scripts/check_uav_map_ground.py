#!/usr/bin/env python3
"""Read-only geometry check on a known unobstructed ground patch in air_ground_v1."""
import argparse
import json
from pathlib import Path
import time

import numpy as np


def ground_summary(points, transform, roi, tolerance=.02):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    valid = np.all(np.isfinite(points), axis=1) & np.any(points != 0, axis=1)
    mapped = points[valid] @ transform[:3, :3].T + transform[:3, 3]
    x0, x1, y0, y1 = roi
    patch = mapped[(mapped[:, 0] >= x0) & (mapped[:, 0] <= x1)
                   & (mapped[:, 1] >= y0) & (mapped[:, 1] <= y1)]
    if not len(patch):
        raise ValueError('no returns in the known ground patch')
    error = patch[:, 2]  # Known map ground is z=0; no fitting or offset removal.
    return dict(points=len(error), median_error_m=float(np.median(error)),
                max_abs_error_m=float(np.max(abs(error))),
                within_tolerance=bool(np.all(abs(error) <= tolerance)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--samples', type=int, default=5)
    parser.add_argument('--timeout', type=float, default=45)
    parser.add_argument('--height-range', nargs=2, type=float, default=[0, .1])
    parser.add_argument('--ground-roi', nargs=4, type=float, default=[6, 10, 1.5, 4])
    args = parser.parse_args()
    import rospy
    import tf2_ros
    from tf.transformations import quaternion_matrix
    from tf2_msgs.msg import TFMessage
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs import point_cloud2
    from prometheus_msgs.msg import UAVState

    rospy.init_node('check_uav_map_ground', anonymous=True)
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    authorities, state = {}, [None]
    def record_tf(msg):
        authority = msg._connection_header.get('callerid', '')
        for tr in msg.transforms:
            edge = (tr.header.frame_id.lstrip('/'), tr.child_frame_id.lstrip('/'))
            authorities.setdefault(edge, set()).add(authority)
    subscriptions = [rospy.Subscriber(topic, TFMessage, record_tf) for topic in ('/tf', '/tf_static')]
    subscriptions.append(rospy.Subscriber('/uav1/prometheus/state', UAVState,
                                          lambda msg: state.__setitem__(0, msg)))
    result = dict(status='FAIL', expected_ground_z_m=0, tolerance_m=.02,
                  ground_roi=args.ground_roi, height_range=args.height_range, samples=[])
    deadline, previous = time.monotonic() + args.timeout, 0
    while not rospy.is_shutdown() and time.monotonic() < deadline and len(result['samples']) < args.samples:
        try:
            cloud = rospy.wait_for_message('/uav1/livox/lidar', PointCloud2, timeout=2)
            stamp = cloud.header.stamp
            if stamp.to_sec() <= previous:
                continue
            previous = stamp.to_sec()
            base = buffer.lookup_transform('map', 'uav1/base_link', stamp, rospy.Duration(1))
            height = base.transform.translation.z
            if not args.height_range[0] <= height <= args.height_range[1]:
                continue
            if state[0] is None or np.linalg.norm(state[0].velocity) > .10:
                continue
            sensor = buffer.lookup_transform('map', cloud.header.frame_id.lstrip('/'), stamp, rospy.Duration(1))
            p, q = sensor.transform.translation, sensor.transform.rotation
            transform = quaternion_matrix([q.x, q.y, q.z, q.w])
            transform[:3, 3] = [p.x, p.y, p.z]
            points = list(point_cloud2.read_points(cloud, field_names=('x', 'y', 'z')))
            sample = ground_summary(points, transform, args.ground_roi)
            sample.update(stamp_s=stamp.to_sec(), public_base_z_m=height)
            result['samples'].append(sample)
            print(json.dumps(sample), flush=True)
        except (rospy.ROSException, tf2_ros.TransformException, ValueError) as error:
            result['last_wait_reason'] = str(error)
    expected = {('world', 'map'): '/sim_world_to_map',
                ('map', 'uav1/odom'): '/sim_localization_uav1',
                ('uav1/odom', 'uav1/base_link'): '/uav_control_main_1',
                ('uav1/base_link', 'uav1/lidar_link'): '/uav_control_main_1',
                ('map', 'ground/odom'): '/sim_localization_ground'}
    result['authorities'] = {' -> '.join(edge): sorted(authorities.get(edge, set())) for edge in expected}
    result['single_expected_authorities'] = all(authorities.get(edge) == {owner} for edge, owner in expected.items())
    if (len(result['samples']) == args.samples and result['single_expected_authorities']
            and all(sample['within_tolerance'] for sample in result['samples'])):
        result['status'] = 'PASS'
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    return 0 if result['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
