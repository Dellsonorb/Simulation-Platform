"""Camera-centered Ground observation, independent of grasp continuation IK."""
import itertools
import math

import numpy as np

from .grasp import quaternion_matrix


def observation_poses(target, size, tcp_from_camera):
    """Six fixed views: three top-surface depths and two optical rolls.

    0.35--0.55 m fits the 0.24 m long top in the calibrated D435 image;
    actual calibration/FOV is checked separately. No outcome-based reseeding.
    """
    target, size, mount = (np.asarray(v, dtype=float) for v in (target, size, tcp_from_camera))
    if (target.shape != (4,) or size.shape != (3,) or mount.shape != (4, 4)
            or not all(np.isfinite(v).all() for v in (target, size, mount))
            or np.any(size <= 0) or not np.allclose(mount[3], [0, 0, 0, 1])
            or not np.allclose(mount[:3, :3].T @ mount[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(mount[:3, :3]), 1.)):
        raise ValueError('invalid target geometry or camera mounting transform')
    views = []
    for depth in (.45, .55, .35):
        for roll in (0., math.pi):
            yaw = target[3]+roll
            camera = np.eye(4)
            camera[:3, :3] = np.array([[math.sin(yaw), -math.cos(yaw), 0],
                                       [-math.cos(yaw), -math.sin(yaw), 0], [0, 0, -1]])
            camera[:3, 3] = [*target[:2], target[2]+size[2]/2.+depth]
            views.append(camera @ np.linalg.inv(mount))
    return views


def target_in_fov(map_from_camera, target, size, calibration, image_size):
    """Conservative all-corner positive-depth image test, not an occlusion claim."""
    target, size, k = (np.asarray(v, dtype=float) for v in (target, size, calibration))
    rotation = np.array([[math.cos(target[3]), -math.sin(target[3]), 0],
                         [math.sin(target[3]), math.cos(target[3]), 0], [0, 0, 1]])
    corners = np.array(list(itertools.product((-1, 1), repeat=3))) * size/2.
    corners = corners @ rotation.T + target[:3]
    camera = np.linalg.inv(map_from_camera)
    points = corners @ camera[:3, :3].T + camera[:3, 3]
    if not np.isfinite(points).all() or np.any(points[:, 2] <= .12) or np.any(points[:, 2] >= 2.5):
        return False
    uv = points @ k.reshape(3, 3).T
    uv = uv[:, :2]/uv[:, 2, None]
    return bool(np.all(uv >= 2.) and np.all(uv < np.asarray(image_size)-2.))


def observe_from_camera_poses(owner, aerial_target, error_type):
    """Use fresh surface evidence to aim; only accepted RGB-D may refine grasp."""
    import rospy
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import CameraInfo
    from tf.transformations import quaternion_from_matrix
    from .flight import observation_is_fresh

    owner._publish_status('GROUND_OBSERVE', mode='camera_centered_v1')
    opening = owner._open_gripper()
    # A useful existing view need not be destroyed by moving the camera.
    try:
        return owner._wait_for_ground_target(opening, timeout=2.)
    except error_type as error:
        owner._publish_status('GROUND_VIEW_REJECTED', view='current', reason=str(error))
    target = list(aerial_target)
    with owner._lock:
        cue = owner._ground_surface_cue
    if cue is not None and observation_is_fresh(cue.header.stamp.to_sec(), rospy.Time.now().to_sec(),
                                               owner._ground_observation_max_age,
                                               cue.header.frame_id, owner._map_frame):
        xy = [cue.point.x, cue.point.y]
        if all(math.isfinite(v) for v in xy):
            target[:2] = xy  # aiming cue only: no cuboid height/yaw inferred from partial surface
            owner._publish_status('GROUND_VIEW_AIM', source='measured_surface', xy=xy,
                                  observation_stamp=cue.header.stamp.to_sec())
    group = owner._initialize_moveit()
    info = rospy.wait_for_message('/ground/d435/color/camera_info', CameraInfo, timeout=5.)
    camera_frame = info.header.frame_id.lstrip('/')
    transform = owner._tf_buffer.lookup_transform(owner._end_effector_link, camera_frame,
                                                 rospy.Time(0), rospy.Duration(.5))
    rotation = transform.transform.rotation
    mount = np.eye(4)
    mount[:3, :3] = quaternion_matrix((rotation.x, rotation.y, rotation.z, rotation.w))
    translation = transform.transform.translation
    mount[:3, 3] = [translation.x, translation.y, translation.z]
    for index, tcp in enumerate(observation_poses(target, owner._target_size, mount)):
        if not target_in_fov(tcp @ mount, target, owner._target_size, info.K, [info.width, info.height]):
            owner._publish_status('GROUND_VIEW_REJECTED', view=index, reason='predicted_target_outside_fov')
            continue
        pose = PoseStamped()
        pose.header.frame_id = owner._map_frame
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = tcp[:3, 3]
        q = quaternion_from_matrix(tcp)
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
        planning_pose = owner._transform_pose(pose, group.get_planning_frame())
        if getattr(owner, '_execution_clearance', False):
            group.set_start_state(owner._check_full_robot_state('camera observation start'))
        else:
            group.set_start_state_to_current_state()
        group.set_pose_target(planning_pose, owner._end_effector_link)
        planned = group.plan()  # collision-aware observation plan; no grasp IK prerequisite
        success = bool(planned[0]) if isinstance(planned, tuple) else True
        trajectory = planned[1] if isinstance(planned, tuple) else planned
        code = getattr(planned[3], 'val', None) if isinstance(planned, tuple) and len(planned) > 3 else None
        group.clear_pose_targets()
        owner._publish_status('GROUND_VIEW_PLAN', view=index, success=success, error_code=code,
                              tcp_map=list(map(float, tcp[:3, 3])), aim_map=target)
        if not success or not trajectory.joint_trajectory.points:
            continue
        if getattr(owner, '_execution_clearance', False):
            owner._execute_checked_arm(trajectory, 'camera observation')
        else:
            if not group.execute(trajectory, wait=True):
                raise error_type('camera observation trajectory execution failed')
            group.stop()
        owner._verify_tcp_pose(planning_pose, 'camera observation')
        try:
            return owner._wait_for_ground_target(opening)
        except error_type as error:
            owner._publish_status('GROUND_VIEW_REJECTED', view=index, reason=str(error))
    raise error_type('no valid near-field D435 observation in six bounded camera views')
