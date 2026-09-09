"""Perceived cuboid MoveIt scene diffs; no runtime or simulator side effects.

Caller contract: fetch a current full scene (world geometry, ACM, robot state
and attached objects) before each change, and supply the complete RobotModel
link list, not merely the arm group's links. Apply the returned diff with the
existing ApplyPlanningScene service and check its success before planning.
The helpers preserve unrelated scene data by omission and merge the ACM;
they do not replace the BUNKER/AUBO/AG95 collision model or load environment GT.

Lifecycle: world_target_diff(accepted aerial pose), observation, then the same
update with the accepted refined pose. Enable finger_contact_diff only for
the grasp phase. After fresh real grasp confirmation, attach_target_diff uses
a fresh measured TCP pose in the target's frame. It adds a planning payload,
not a physical constraint. Keep that payload in all lift/start-state requests
via planning_start_state. On an abandoned grasp, disable finger contact.
Caller owns perception acceptance, measurement freshness, execution, release
cleanup, and serialized scene writes. Online behavior is not verified here.
"""

from collections import namedtuple
import copy
import math

import numpy as np
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (AllowedCollisionEntry, AttachedCollisionObject,
                             CollisionObject, PlanningScene)
from shape_msgs.msg import SolidPrimitive
from tf.transformations import quaternion_from_matrix

from .grasp import (GraspError, check_target_feasibility,
                    conservative_jaw_opening, quaternion_matrix)


TARGET_ID = "perceived_pick_target"
# MoveIt retains the pads as separate collision links, unlike Gazebo's fixed
# joint lumping. These four links are the physical finger contact assembly;
# the knuckles, gripper body, wrist, arm and chassis never receive exemptions.
FINGER_LINKS = ("ground/left_finger", "ground/right_finger",
                "ground/left_finger_pad", "ground/right_finger_pad")
Preshape = namedtuple("Preshape", "command_position closure_limit required_opening predicted_opening")
ContactGeometry = namedtuple("ContactGeometry", "q_contact pad_edge delta")


class SceneError(RuntimeError):
    pass


def _dimensions(values):
    try:
        result = [float(value) for value in values]
    except (ValueError, TypeError, OverflowError) as error:
        raise SceneError("target dimensions must be positive and finite") from error
    if len(result) != 3 or not all(math.isfinite(v) and v > 0.0 for v in result):
        raise SceneError("target dimensions must be positive and finite")
    return result


def _pose(value):
    result = copy.deepcopy(value)
    p, q = result.position, result.orientation
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in values):
        raise SceneError("pose must be finite")
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if not math.isfinite(norm) or norm <= 1e-12:
        raise SceneError("pose quaternion has invalid norm")
    q.x, q.y, q.z, q.w = [v/norm for v in (q.x, q.y, q.z, q.w)]
    return result


def _frame(frame):
    if not isinstance(frame, str) or not frame.strip():
        raise SceneError("pose frame must be explicit")


def _pose_matrix(value):
    checked = _pose(value)
    p, q = checked.position, checked.orientation
    result = np.eye(4)
    result[:3, :3] = quaternion_matrix((q.x, q.y, q.z, q.w))
    result[:3, 3] = [p.x, p.y, p.z]
    return result


def _diff():
    scene = PlanningScene()
    scene.is_diff = True
    # An empty full RobotState would discard existing attachments.
    scene.robot_state.is_diff = True
    return scene


def _target_acm(current, robot_link_names, object_id, fingers_allowed):
    links = list(robot_link_names)
    if (not links or len(links) != len(set(links)) or
            any(not isinstance(link, str) or not link for link in links) or
            not set(("ground/base_link",) + FINGER_LINKS).issubset(links)):
        raise SceneError("complete robot link list including chassis and fingers required")
    if not isinstance(object_id, str) or not object_id or object_id in links:
        raise SceneError("target object id is invalid")
    old = current.allowed_collision_matrix
    count = len(old.entry_names)
    if (len(set(old.entry_names)) != count or len(old.entry_values) != count or
            any(len(row.enabled) != count for row in old.entry_values) or
            len(old.default_entry_names) != len(old.default_entry_values) or
            len(set(old.default_entry_names)) != len(old.default_entry_names)):
        raise SceneError("existing allowed collision matrix is malformed")
    if any(old.entry_values[i].enabled[j] != old.entry_values[j].enabled[i]
           for i in range(count) for j in range(count)):
        raise SceneError("existing allowed collision matrix is asymmetric")
    defaults = dict(zip(old.default_entry_names, old.default_entry_values))
    names = list(old.entry_names)
    for name in links + [obj.id for obj in current.world.collision_objects] + [
            obj.object.id for obj in current.robot_state.attached_collision_objects] + [object_id]:
        if name and name not in names:
            names.append(name)
    old_indices = {name: index for index, name in enumerate(old.entry_names)}
    acm = copy.deepcopy(old)
    acm.entry_names = names
    acm.entry_values = []
    for first in names:
        row = []
        for second in names:
            if object_id in (first, second):
                other = second if first == object_id else first
                allowed = bool(fingers_allowed and other in FINGER_LINKS)
            elif first in old_indices and second in old_indices:
                allowed = old.entry_values[old_indices[first]].enabled[old_indices[second]]
            else:
                # MoveIt combines two defaults with AND; one default applies
                # alone. Preserve these effective pairs when extending rows.
                selected = [defaults[name] for name in (first, second) if name in defaults]
                allowed = bool(selected and all(selected))
            row.append(allowed)
        acm.entry_values.append(AllowedCollisionEntry(row))
    if object_id in acm.default_entry_names:
        acm.default_entry_values[acm.default_entry_names.index(object_id)] = False
    else:
        acm.default_entry_names.append(object_id)
        acm.default_entry_values.append(False)
    return acm


def _world_target(current, object_id):
    if any(obj.object.id == object_id for obj in current.robot_state.attached_collision_objects):
        raise SceneError("target is already attached")
    matches = [obj for obj in current.world.collision_objects if obj.id == object_id]
    if len(matches) != 1:
        raise SceneError("exactly one perceived world target is required")
    target = matches[0]
    if (len(target.primitives) != 1 or len(target.primitive_poses) != 1 or
            target.primitives[0].type != SolidPrimitive.BOX or target.meshes or target.planes):
        raise SceneError("perceived target must be one cuboid")
    _dimensions(target.primitives[0].dimensions)
    _frame(target.header.frame_id)
    _pose(target.pose)
    _pose(target.primitive_poses[0])
    return target


def world_target_diff(current, accepted_pose, dimensions, robot_link_names,
                      object_id=TARGET_ID):
    """Add/update accepted geometry and disable all target contact exemptions."""
    if any(obj.object.id == object_id for obj in current.robot_state.attached_collision_objects):
        raise SceneError("cannot overwrite attached target with world perception")
    _frame(accepted_pose.header.frame_id)
    target = CollisionObject()
    target.header = copy.deepcopy(accepted_pose.header)
    target.id = object_id
    target.operation = CollisionObject.ADD
    target.pose.orientation.w = 1.0
    target.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=_dimensions(dimensions))]
    target.primitive_poses = [_pose(accepted_pose.pose)]
    change = _diff()
    change.world.collision_objects = [target]
    change.allowed_collision_matrix = _target_acm(current, robot_link_names, object_id, False)
    return change


def finger_contact_diff(current, robot_link_names, enabled, object_id=TARGET_ID):
    """Allow only the four physical finger-assembly/target grasp-contact pairs."""
    _world_target(current, object_id)
    if not isinstance(enabled, bool):
        raise SceneError("finger contact flag must be boolean")
    change = _diff()
    change.allowed_collision_matrix = _target_acm(current, robot_link_names, object_id, enabled)
    return change


def attach_target_diff(current, measured_tcp_pose, tcp_link, robot_link_names,
                       grasp_confirmed, object_id=TARGET_ID):
    """Express the accepted world cuboid in measured TCP coordinates and attach.

    T_tcp_shape = inverse(T_frame_tcp_measured) * T_frame_object * T_object_shape.
    A same-frame measurement is required; the caller owns TF lookup/freshness.
    No target measurement is replaced by commanded TCP or nominal geometry.
    """
    if grasp_confirmed is not True:
        raise SceneError("real grasp confirmation required before planning attachment")
    return _attachment_geometry_diff(current, measured_tcp_pose, tcp_link, robot_link_names, object_id)


def _attachment_geometry_diff(current, measured_tcp_pose, tcp_link, robot_link_names,
                              object_id=TARGET_ID):
    """Geometry-only construction; real attachment uses the confirmation gate above."""
    target = _world_target(current, object_id)
    if measured_tcp_pose.header.frame_id != target.header.frame_id:
        raise SceneError("measured TCP and target frames must match")
    links = list(robot_link_names)
    if tcp_link not in links:
        raise SceneError("attachment TCP link is not in the robot model")
    relative = (np.linalg.inv(_pose_matrix(measured_tcp_pose.pose)) @
                _pose_matrix(target.pose) @ _pose_matrix(target.primitive_poses[0]))
    quaternion = quaternion_from_matrix(relative)
    local_pose = Pose()
    local_pose.position.x, local_pose.position.y, local_pose.position.z = relative[:3, 3].tolist()
    (local_pose.orientation.x, local_pose.orientation.y,
     local_pose.orientation.z, local_pose.orientation.w) = quaternion.tolist()
    attachment = AttachedCollisionObject()
    attachment.link_name = tcp_link
    attachment.touch_links = list(FINGER_LINKS)
    attachment.object = copy.deepcopy(target)
    attachment.object.header = copy.deepcopy(measured_tcp_pose.header)
    attachment.object.header.frame_id = tcp_link
    attachment.object.pose = Pose()
    attachment.object.pose.orientation.w = 1.0
    attachment.object.primitive_poses = [local_pose]
    attachment.object.operation = CollisionObject.ADD
    change = _diff()
    # Native MoveIt attachment ADD atomically removes the same-ID world
    # object. A redundant world REMOVE is processed afterwards and makes
    # ApplyPlanningScene report failure despite a successful attachment.
    change.robot_state.attached_collision_objects = [attachment]
    change.allowed_collision_matrix = _target_acm(current, links, object_id, True)
    return change


def planning_start_state(scene_robot_state, measured_joint_state):
    """Copy payload/multi-DOF state, use measured joints, and preserve the scene."""
    names, positions = measured_joint_state.name, measured_joint_state.position
    if (not names or len(names) != len(positions) or len(set(names)) != len(names) or
            any(not name for name in names) or not all(math.isfinite(v) for v in positions)):
        raise SceneError("measured joint positions are incomplete or nonfinite")
    state = copy.deepcopy(scene_robot_state)
    state.joint_state = copy.deepcopy(measured_joint_state)
    state.is_diff = True
    return state


def ag95_contact_geometry(target_size, maximum_opening, maximum_joint_position, open_pad_edge):
    """URDF linkage geometry at nominal pad contact, not contact confirmation.

    The configured pad edge is above the TCP with fully open fingers. Closing
    translates the parallel pads along TCP +X (down in the top grasp). Keep
    that open calibration and subtract its 55-mm-link displacement; do not
    fit insertion depth or change the requested overlap. The physical target
    span sets contact, whereas the opening margin belongs only to preshape.
    """
    try:
        feasibility = check_target_feasibility(target_size, maximum_opening, 0.)
        conservative_jaw_opening(0., maximum_opening, maximum_joint_position)
        edge = float(open_pad_edge)
        alpha, length = math.radians(44.691), .055  # Rendered AG95 URDF.
        cosine = math.cos(alpha) + (feasibility.grasp_span - float(maximum_opening)) / (2 * length)
        if not feasibility.feasible or not -1. <= cosine <= 1.:
            raise SceneError("target has no calibrated AG95 pad-contact position")
        # The exactly-open boundary is known analytically; avoid acos roundoff.
        joint = 0. if feasibility.grasp_span == float(maximum_opening) else math.acos(cosine) - alpha
        delta = length * (math.sin(alpha + joint) - math.sin(alpha))
        pad_edge = edge - delta
        if (not math.isfinite(edge) or edge <= 0. or
                not 0. <= joint <= float(maximum_joint_position) or pad_edge <= 0.):
            raise SceneError("AG95 contact geometry exceeds calibrated stroke or pad-edge range")
    except (GraspError, ValueError, TypeError, OverflowError) as error:
        raise SceneError("invalid target or AG95 contact calibration") from error
    return ContactGeometry(joint, pad_edge, delta)


def required_opening_preshape(target_size, maximum_opening, maximum_joint_position,
                             opening_margin, joint_tracking_tolerance):
    """Invert the existing conservative opening bound, reserving tracking room.

    This is a candidate actuator command, not a collision or opening certificate.
    The actual conservative aperture must still be strictly greater than the
    required opening. At exactly +tracking_tolerance, equality can occur and
    must fail that existing strict check. No fitted collision margin is added.
    """
    try:
        feasibility = check_target_feasibility(target_size, maximum_opening, opening_margin)
        conservative_jaw_opening(0.0, maximum_opening, maximum_joint_position)
        tracking = float(joint_tracking_tolerance)
        closure_limit = float(maximum_joint_position) * (1.0 - feasibility.required_opening/float(maximum_opening))
        command = closure_limit - tracking
        if (not feasibility.feasible or not math.isfinite(tracking) or
                tracking < 0.0 or command < 0.0):
            raise SceneError("target opening and tracking room exceed calibrated stroke")
        opening = conservative_jaw_opening(command, maximum_opening, maximum_joint_position)
    except (GraspError, ValueError, TypeError, OverflowError) as error:
        raise SceneError("invalid target or gripper calibration") from error
    return Preshape(command, closure_limit, feasibility.required_opening, opening)
