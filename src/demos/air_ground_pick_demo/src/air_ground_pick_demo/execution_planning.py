"""Bounded, shared SIM whole-manipulation screening and checked arm dispatch.

Preview uses accepted perception and public base TF. Temporary attachments are
planning geometry only. No search function can send an actuator command.
"""
import copy
from contextlib import contextmanager
import math
import threading
import time

import rospy
import tf2_ros
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from control_msgs.msg import FollowJointTrajectoryGoal, JointTrajectoryControllerState
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, MoveItErrorCodes
from moveit_msgs.srv import GetCartesianPathRequest, GetPositionIK, GetPositionIKRequest, GetStateValidityRequest
from trajectory_msgs.msg import JointTrajectoryPoint

from . import execution_clearance as ec, manipulation_scene as ms
from .approach import transform_is_fresh
from .grasp import zero_terminal_motion

ARM_NAMES = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
             'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint')
HOME_SEED = (0., -.8, 1.6, .8, math.pi/2., 0.)
ALTERNATE_SEED = (0., -1.2, -1.8, 1.4, -math.pi/2., 0.)
GRIPPER_JOINT = 'left_outer_knuckle_joint'
CHECK_WALL_TIMEOUT_S = 60.
JOINT_SAMPLE_STEP_RAD = .005


class PlanningRejected(RuntimeError):
    """A completed geometric/planning check found this branch infeasible."""


def require_solver_success(code, label):
    if code == MoveItErrorCodes.SUCCESS:
        return
    geometric_or_solver = {
        MoveItErrorCodes.PLANNING_FAILED, MoveItErrorCodes.INVALID_MOTION_PLAN,
        MoveItErrorCodes.TIMED_OUT, MoveItErrorCodes.START_STATE_IN_COLLISION,
        MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS, MoveItErrorCodes.GOAL_IN_COLLISION,
        MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS, MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED,
        MoveItErrorCodes.NO_IK_SOLUTION}
    if code in geometric_or_solver:
        raise PlanningRejected('%s: solver rejected branch (code %s)' % (label, code))
    raise RuntimeError('%s: MoveIt interface/configuration failure (code %s)' % (label, code))


@contextmanager
def temporary_scene(owner):
    """Restore full world, ACM, transforms and attachments even after failure."""
    original = copy.deepcopy(owner._get_manipulation_scene())
    try:
        yield original
    finally:
        # MoveIt's full scene setter does not clear unspecified attachments.
        # Explicitly remove newly introduced hypotheses before restoring the
        # snapshot; otherwise it can leave both a world object and a payload.
        current = owner._get_manipulation_scene()
        original_ids = {a.object.id for a in original.robot_state.attached_collision_objects}
        introduced = [a for a in current.robot_state.attached_collision_objects if a.object.id not in original_ids]
        if introduced:
            remove = ms._diff()
            remove.robot_state.attached_collision_objects = [AttachedCollisionObject(
                link_name=a.link_name, object=CollisionObject(id=a.object.id, operation=CollisionObject.REMOVE))
                for a in introduced]
            owner._apply_manipulation_scene(remove)
        original.is_diff = False
        original.robot_state.is_diff = False
        owner._apply_manipulation_scene(original)


def install_guard(owner):
    scene = owner._get_manipulation_scene()
    description = rospy.get_param('/robot_description')
    owner._apply_manipulation_scene(ec.chassis_clearance_diff(
        scene, description, owner._scene_robot_links))


def check_state(owner, state, label, require_payload=False):
    if require_payload and not any(a.object.id == ms.TARGET_ID for a in state.attached_collision_objects):
        raise RuntimeError(label + ': perceived payload absent from full state')
    request = GetStateValidityRequest(robot_state=state, group_name='')
    response = owner._state_validity(request)  # Transport/interface errors must propagate.
    if not response.valid:
        contacts = ['%s/%s' % (c.contact_body_1, c.contact_body_2) for c in response.contacts]
        raise PlanningRejected('%s: full robot collision %s' % (label, contacts))
    return state


def branch_seeds(measured, active_names, rm_seed):
    if set(active_names) != set(ARM_NAMES):
        raise ValueError('execution search requires the canonical six AUBO joints')
    seed = HOME_SEED if rm_seed is None else tuple(float(q) for q in rm_seed)
    if len(seed) != 6 or not all(math.isfinite(q) for q in seed):
        raise ValueError('original RM IK seed must contain six finite canonical joint values')
    return [(yaw, name, state) for yaw in (0., math.pi) for name, state in (
        ('measured', copy.deepcopy(measured)),
        ('rm_or_home', ec.state_at_positions(measured, ARM_NAMES, seed)),
        ('alternate', ec.state_at_positions(measured, ARM_NAMES, ALTERNATE_SEED)))]


def endpoint(start, trajectory):
    jt = trajectory.joint_trajectory
    return ec.state_at_positions(start, jt.joint_names, jt.points[-1].positions)


def joint_bridge(start, names, positions):
    current = dict(zip(start.joint_state.name, start.joint_state.position))
    initial = [current[name] for name in names]
    count = max(1, int(math.ceil(max(abs(a-b) for a, b in zip(initial, positions)) /
                               JOINT_SAMPLE_STEP_RAD)))
    for i in range(count + 1):
        yield ec.state_at_positions(start, names, [a+(b-a)*i/count for a, b in zip(initial, positions)])


def check_samples(owner, samples, label, require_payload=False):
    """Bound a whole sequence, including a stalled read-only validity service.

    On timeout no later branch or action is allowed; a pending read-only RPC
    may finish in its daemon thread and cannot issue another call afterwards.
    """
    done, stopped = threading.Event(), threading.Event()
    result = {'count': 0}
    def work():
        try:
            for state in samples:
                if stopped.is_set():
                    return
                check_state(owner, state, label, require_payload)
                result['count'] += 1
        except BaseException as error:
            result['error'] = error
        finally:
            done.set()
    worker = threading.Thread(target=work, daemon=True)
    worker.start()
    if not done.wait(CHECK_WALL_TIMEOUT_S):
        stopped.set()
        raise RuntimeError(label + ': full trajectory checking exceeded 60-second wall budget')
    if 'error' in result:
        raise result['error']
    if result['count'] == 0:
        raise RuntimeError(label + ': empty state sequence')
    return result['count']


def checked_curve(owner, start, trajectory, label, held_desired=None, require_payload=False):
    def samples():
        yield start
        if held_desired is not None:
            yield from joint_bridge(start, trajectory.joint_trajectory.joint_names, held_desired.positions)
        yield from ec.controller_samples(start, trajectory, held_desired=held_desired)
    return check_samples(owner, samples(), label, require_payload)


def loaded_articulations(owner, start):
    q = dict(zip(start.joint_state.name, start.joint_state.position))[GRIPPER_JOINT]
    if not math.isfinite(q) or not 0. <= q <= ec.OBSERVED_LOADED_JOINT_MAX:
        raise RuntimeError('measured AG95 closure exceeds the checked loaded range')
    geometry = ms.ag95_contact_geometry(owner._target_size, owner._maximum_gripper_opening,
                                       owner._maximum_gripper_joint, owner._finger_pad_lower_edge_offset)
    samples = ec.closure_positions(min(q, geometry.q_contact), geometry.q_contact, owner._cartesian_eef_step)
    # Keep actual geometry and nominal contact explicitly even if the sampling
    # grid does not land on them. The real attached object is unchanged.
    return sorted(set(samples + [q, geometry.q_contact]))


def checked_loaded_lift(owner, start, trajectory, label, held_desired):
    articulations = loaded_articulations(owner, start)
    def states():
        for articulation in articulations:
            state = ec.state_at_positions(start, [GRIPPER_JOINT], [articulation])
            yield state
            yield from joint_bridge(state, trajectory.joint_trajectory.joint_names, held_desired.positions)
            yield from ec.controller_samples(state, trajectory, held_desired=held_desired)
    return check_samples(owner, states(), label + ' loaded articulation range', require_payload=True)


def cartesian(owner, start, target, label):
    request = GetCartesianPathRequest()
    request.header, request.start_state = copy.deepcopy(target.header), copy.deepcopy(start)
    request.group_name, request.link_name = owner._move_group_name, owner._end_effector_link
    request.waypoints = [target.pose]
    request.max_step, request.jump_threshold, request.avoid_collisions = owner._cartesian_eef_step, 0., True
    response = owner._cartesian_path(request)
    require_solver_success(response.error_code.val, label)
    if (not math.isfinite(response.fraction) or
            response.fraction < owner._cartesian_min_fraction or not response.solution.joint_trajectory.points):
        raise PlanningRejected('%s: incomplete Cartesian path (code %s)' % (label, response.error_code.val))
    trajectory = owner._move_group.retime_trajectory(
        start, response.solution, owner._cartesian_velocity_scaling, owner._cartesian_velocity_scaling)
    if not trajectory.joint_trajectory.points:
        raise PlanningRejected(label + ': Cartesian retiming failed')
    return zero_terminal_motion(trajectory)


def poses(owner, target, frame, stamp, yaw_offset):
    turned = list(target)
    turned[3] += yaw_offset
    generated = owner._generate_ground_grasp(turned)
    values = [owner._pose_message(value, frame, stamp) for value in
              (generated.pregrasp, generated.grasp, generated.lift)]
    planning_frame = owner._move_group.get_planning_frame()
    return tuple(value if frame == planning_frame else owner._transform_pose(value, planning_frame)
                 for value in values)


def _search_branch(owner, measured, seed, messages, actual, held_desired, diagnostic):
    pregrasp, grasp, lift = messages
    diagnostic['stage'] = 'grasp_ik'
    request = GetPositionIKRequest()
    request.ik_request.group_name, request.ik_request.ik_link_name = owner._move_group_name, owner._end_effector_link
    request.ik_request.pose_stamped, request.ik_request.robot_state = grasp, seed
    request.ik_request.avoid_collisions = True
    request.ik_request.timeout = rospy.Duration(owner._planning_time)
    response = owner._execution_ik(request)
    require_solver_success(response.error_code.val, 'grasp IK')
    solved = dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))
    grasp_state = ec.state_at_positions(measured, ARM_NAMES, [solved[name] for name in ARM_NAMES])
    check_state(owner, grasp_state, 'grasp IK whole robot')
    diagnostic['stage'] = 'reverse_approach'
    reverse = cartesian(owner, grasp_state, pregrasp, 'reverse approach')
    checked_curve(owner, grasp_state, reverse, 'reverse approach')
    pregrasp_state = endpoint(grasp_state, reverse)
    approach = None
    if actual:
        diagnostic['stage'] = 'pregrasp_transit'
        group = owner._move_group
        group.set_start_state(measured)
        q = dict(zip(pregrasp_state.joint_state.name, pregrasp_state.joint_state.position))
        group.set_joint_value_target({name: q[name] for name in ARM_NAMES})
        planned = group.plan()
        if isinstance(planned, tuple) and len(planned) > 3:
            require_solver_success(planned[3].val, 'pregrasp transit')
        success = bool(planned[0]) if isinstance(planned, tuple) else True
        approach = planned[1] if isinstance(planned, tuple) else planned
        group.clear_pose_targets()
        if not success or not approach.joint_trajectory.points:
            raise PlanningRejected('pregrasp transit planning failed')
        approach = zero_terminal_motion(approach)
        checked_curve(owner, measured, approach, 'pregrasp transit', held_desired)
        pregrasp_state = endpoint(measured, approach)
    diagnostic['stage'] = 'forward_approach'
    forward = cartesian(owner, pregrasp_state, grasp, 'forward approach')
    checked_curve(owner, pregrasp_state, forward, 'forward approach')
    grasp_state = endpoint(pregrasp_state, forward)
    diagnostic['stage'] = 'closure'
    owner._apply_manipulation_scene(ms.finger_contact_diff(
        owner._get_manipulation_scene(), owner._scene_robot_links, True))
    geometry = ms.ag95_contact_geometry(owner._target_size, owner._maximum_gripper_opening,
                                       owner._maximum_gripper_joint, owner._finger_pad_lower_edge_offset)
    measured_q = dict(zip(grasp_state.joint_state.name, grasp_state.joint_state.position))[GRIPPER_JOINT]
    closing = ec.closure_positions(measured_q, geometry.q_contact, owner._cartesian_eef_step)
    check_samples(owner, (ec.state_at_positions(grasp_state, [GRIPPER_JOINT], [q]) for q in closing),
                  'physical closure')
    diagnostic['stage'] = 'attached_lift'
    owner._apply_manipulation_scene(ec.predicted_payload_diff(
        owner._get_manipulation_scene(), grasp, owner._end_effector_link, owner._scene_robot_links))
    attached = owner._get_manipulation_scene().robot_state.attached_collision_objects
    # Check the same vertical lift across every sampled loaded articulation,
    # explicitly including nominal contact and the measured-history maximum.
    loaded = sorted(set([geometry.q_contact] + [q for q in closing if q >= geometry.q_contact]))
    for q in loaded:
        lift_state = ec.state_at_positions(grasp_state, [GRIPPER_JOINT], [q])
        lift_state.attached_collision_objects = copy.deepcopy(attached)
        trajectory = cartesian(owner, lift_state, lift, 'attached lift')
        checked_curve(owner, lift_state, trajectory, 'attached lift', require_payload=True)
    diagnostic.update(stage='accepted', reason='whole manipulation chain checked', loaded_joint_max=max(loaded))
    return dict(pregrasp=pregrasp, grasp=grasp, lift=lift, approach=approach)


def search(owner, target, frame, stamp, rm_seed, actual=False, held_desired=None, resolved_poses=None):
    owner._initialize_moveit()
    # Freeze accepted-frame transforms before bounded IK/planning work can
    # outlive the TF buffer. Actual execution can resolve these before preshape.
    if resolved_poses is None:
        resolved_poses = {offset: poses(owner, target, frame, stamp, offset) for offset in (0., math.pi)}
    if getattr(owner, '_execution_ik', None) is None:
        rospy.wait_for_service('/compute_ik', timeout=owner._moveit_server_timeout)
        rospy.wait_for_service('/compute_cartesian_path', timeout=owner._moveit_server_timeout)
        owner._execution_ik = rospy.ServiceProxy('/compute_ik', GetPositionIK)
    measured = owner._robot_state_from_joint_feedback()
    if not actual:
        nominal = ms.required_opening_preshape(owner._target_size, owner._maximum_gripper_opening,
            owner._maximum_gripper_joint, owner._opening_margin, owner._gripper_joint_tolerance)
        measured = ec.state_at_positions(measured, [GRIPPER_JOINT], [nominal.command_position])
    attempts = []
    for offset, seed_name, seed in branch_seeds(measured, owner._observation_joint_names, rm_seed):
        diagnostic = dict(attempt=len(attempts)+1, grasp_yaw_offset=offset, seed=seed_name, stage='start')
        attempts.append(diagnostic)
        try:
            with temporary_scene(owner):
                check_state(owner, measured, 'whole robot search start')
                selection = _search_branch(owner, measured, seed, resolved_poses[offset],
                                           actual, held_desired, diagnostic)
        except PlanningRejected as error:
            diagnostic['reason'] = str(error)
            continue
        return dict(feasible=True, reason='whole manipulation chain checked', grasp_yaw_offset=offset,
                    attempts=attempts, selection=selection)
    return dict(feasible=False, reason='six bounded branches rejected', grasp_yaw_offset=None, attempts=attempts)


def fresh_base_transform(owner):
    deadline = time.monotonic() + .5
    reason = 'unavailable'
    while not rospy.is_shutdown():
        if time.monotonic() >= deadline:
            raise RuntimeError('candidate preview public map/base transform is ' + reason)
        try:
            transform = owner._tf_buffer.lookup_transform(owner._map_frame, owner._ground_base_frame,
                                                         rospy.Time(0), rospy.Duration(0.))
            if (time.monotonic() < deadline and transform_is_fresh(transform.header.stamp.to_sec(),
                    rospy.Time.now().to_sec(), owner._ground_tf_max_age)):
                return transform
            reason = 'stale'
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as error:
            reason = 'unavailable: ' + str(error)
        remaining = deadline-time.monotonic()
        if remaining > 0.:
            rospy.rostime.wallsleep(min(.05, remaining))
    raise RuntimeError('ROS shutdown while waiting for candidate preview public map/base transform')


def preview(owner, target_map, candidate, rm_seed):
    if not getattr(owner, '_execution_clearance', False) or not owner._full_robot_manipulation:
        raise RuntimeError('execution clearance preview requires both opt-in modes')
    target = tuple(float(v) for v in target_map)
    base = tuple(float(candidate[name]) for name in ('x', 'y', 'yaw'))
    if len(target) != 4 or not all(math.isfinite(v) for v in target + base):
        raise ValueError('candidate and accepted target must be finite')
    group = owner._initialize_moveit()
    if group.get_planning_frame() != owner._ground_base_frame:
        raise RuntimeError('hypothetical candidate preview requires fixed ground/base_link planning frame')
    transform = fresh_base_transform(owner)
    height = float(transform.transform.translation.z)
    if not math.isfinite(height):
        raise ValueError('candidate preview measured base height is nonfinite')
    dx, dy = target[0]-base[0], target[1]-base[1]
    c, s = math.cos(base[2]), math.sin(base[2])
    predicted = (c*dx+s*dy, -s*dx+c*dy, target[2]-height, target[3]-base[2])
    with temporary_scene(owner):
        accepted = PoseStamped()
        accepted.header.frame_id, accepted.header.stamp = owner._ground_base_frame, rospy.Time.now()
        accepted.pose.position.x, accepted.pose.position.y, accepted.pose.position.z = predicted[:3]
        (accepted.pose.orientation.x, accepted.pose.orientation.y,
         accepted.pose.orientation.z, accepted.pose.orientation.w) = owner._quaternion_from_yaw(predicted[3])
        owner._apply_manipulation_scene(ms.world_target_diff(owner._get_manipulation_scene(), accepted,
                                        owner._target_size, owner._scene_robot_links))
        install_guard(owner)
        result = search(owner, predicted, owner._ground_base_frame, accepted.header.stamp, rm_seed)
    result.pop('selection', None)
    result.update(source='aerial_predicted', arrival_revalidation_required=True,
                  camera_observation_and_transit_guaranteed=False, target_base=list(predicted),
                  candidate_base=list(base), measured_base_z=height)
    return result


def stationary_hold(owner, names):
    if getattr(owner, '_execution_arm_failed', False) or getattr(owner, '_execution_arm_in_flight', False):
        raise RuntimeError('arm action failed or remains pending; execution ownership unavailable')
    namespace = owner._arm_action.rsplit('/', 1)[0]
    deadline = time.monotonic() + owner._arm_action_timeout
    status = rospy.wait_for_message(owner._arm_action + '/status', GoalStatusArray,
                                    timeout=owner._arm_action_timeout)
    if any(item.status in (GoalStatus.PENDING, GoalStatus.ACTIVE, GoalStatus.PREEMPTING, GoalStatus.RECALLING)
           for item in status.status_list):
        raise RuntimeError('arm controller has an active or pending goal; execution ownership unavailable')
    while not rospy.is_shutdown():
        remaining = deadline-time.monotonic()
        if remaining <= 0.:
            raise RuntimeError('arm controller stationary desired hold timeout')
        state = rospy.wait_for_message(namespace + '/state', JointTrajectoryControllerState, timeout=remaining)
        if not transform_is_fresh(state.header.stamp.to_sec(), rospy.Time.now().to_sec(), owner._ground_tf_max_age):
            continue
        desired = state.desired
        n = len(state.joint_names)
        if (len(set(state.joint_names)) != n or set(state.joint_names) != set(names) or
                any(len(values) != n for values in (desired.positions, desired.velocities, desired.accelerations)) or
                not all(math.isfinite(q) for q in list(desired.positions)+list(desired.velocities)+list(desired.accelerations))):
            raise RuntimeError('arm controller desired state is incomplete or nonfinite')
        if any(q != 0. for q in list(desired.velocities)+list(desired.accelerations)):
            continue
        indices = [state.joint_names.index(name) for name in names]
        held = JointTrajectoryPoint(positions=[desired.positions[i] for i in indices],
            velocities=[0.]*len(names), accelerations=[0.]*len(names))
        return owner._robot_state_from_joint_feedback(), held
    raise RuntimeError('ROS shutdown while acquiring stationary arm hold')


def execute_checked(owner, trajectory, label):
    trajectory = copy.deepcopy(trajectory)
    trajectory.joint_trajectory.header.stamp = rospy.Time(0)
    names = trajectory.joint_trajectory.joint_names
    start, held = stationary_hold(owner, names)
    require_payload = label == 'lift'
    if require_payload:
        checked = checked_loaded_lift(owner, start, trajectory, label, held)
    else:
        checked = checked_curve(owner, start, trajectory, label, held, require_payload)
    # A long read-only check cannot make an old measured or desired state fresh.
    fresh, latest = stationary_hold(owner, names)
    if latest.positions != held.positions:
        raise RuntimeError(label + ': controller hold changed during trajectory checking')
    if require_payload:
        initial_range = loaded_articulations(owner, start)
        fresh_range = loaded_articulations(owner, fresh)  # Check the hard cap again before any send.
        if min(fresh_range) < min(initial_range):
            raise RuntimeError(label + ': measured articulation left the checked loaded range')
        articulations = sorted(set(initial_range + fresh_range))
        def bridges():
            for q in articulations:
                state = ec.state_at_positions(fresh, [GRIPPER_JOINT], [q])
                yield from joint_bridge(state, names, latest.positions)
        check_samples(owner, bridges(), label + ' fresh loaded start bridge', require_payload=True)
    else:
        check_samples(owner, joint_bridge(fresh, names, latest.positions), label + ' fresh start bridge')
    client = owner._arm_client
    if not client.wait_for_server(rospy.Duration(owner._arm_action_timeout)):
        raise RuntimeError(label + ': arm controller action unavailable')
    goal = FollowJointTrajectoryGoal(trajectory=trajectory.joint_trajectory)
    owner._publish_status('GROUND_EXECUTION_CHECKED', label=label, checked_states=checked,
        sample_period_s=ec.CONTROLLER_SAMPLE_PERIOD_S, backend='serialized_follow_joint_trajectory',
        previous_action_terminal=True, held_desired_stationary=True, continuous_clearance_proven=False)
    owner._execution_arm_in_flight = True
    try:
        client.send_goal(goal)
        if not client.wait_for_result(rospy.Duration(owner._arm_action_timeout)):
            client.cancel_goal()
            client.wait_for_result(rospy.Duration(owner._arm_action_timeout))
            raise RuntimeError(label + ': arm controller action timed out')
        if client.get_state() != GoalStatus.SUCCEEDED:
            raise RuntimeError(label + ': arm controller action failed: %s' % client.get_state())
        measured = owner._robot_state_from_joint_feedback()
        check_samples(owner, [measured], label + ' measured endpoint', require_payload)
        if require_payload:
            q = dict(zip(measured.joint_state.name, measured.joint_state.position))[GRIPPER_JOINT]
            if q > ec.OBSERVED_LOADED_JOINT_MAX:
                raise RuntimeError(label + ': measured AG95 closure exceeds the checked loaded range')
        owner._publish_status('GROUND_EXECUTION_TERMINAL', label=label, action_state=GoalStatus.SUCCEEDED,
                              measured_endpoint_valid=True, continuous_clearance_proven=False)
    except BaseException:
        owner._execution_arm_failed = True
        raise
    finally:
        owner._execution_arm_in_flight = False
