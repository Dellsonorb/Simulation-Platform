"""Small planning-only chassis envelope and explicit trajectory sampling.

The 12-mm development allowance rounds up the 11.491-mm maximum corresponding
collision-point displacement measured in prior runs05--08. It is not calibrated
uncertainty, a replacement robot model, or a guarantee for other collision pairs.
"""
import copy
import math
import xml.etree.ElementTree as ET

import numpy as np
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from tf.transformations import quaternion_from_euler

from . import manipulation_scene as ms

CHASSIS_GUARD_ID = 'execution_chassis_clearance'
CHASSIS_MARGIN_M = .012
OBSERVED_LOADED_JOINT_MAX = .522  # prior maximum .521259581 rad, rounded up 1 mrad
CONTROLLER_SAMPLE_PERIOD_S = .001  # explicitly configured SIM 1-ms physics loop


def predicted_payload_diff(scene, planned_tcp, tcp_link, robot_links):
    """Hypothetical planning geometry ONLY; never reports/accepts a real grasp.

    Caller must restore its scene before any actuation. Actual attachment still
    calls manipulation_scene.attach_target_diff with fresh confirmation and TF.
    """
    return ms._attachment_geometry_diff(scene, planned_tcp, tcp_link, robot_links)


def chassis_geometry(robot_description):
    """Use the authoritative box and the all-fixed chain from the model root."""
    root = ET.fromstring(robot_description)
    base = root.find("link[@name='ground/base_link']")
    if base is None or len(base.findall('collision')) != 1:
        raise ValueError('one authoritative BUNKER collision shape required')
    collision = base.find('collision')
    box = collision.find('geometry/box')
    if box is None:
        raise ValueError('BUNKER clearance copy requires its authoritative box')
    size = [float(v) for v in box.attrib['size'].split()]
    if len(size) != 3 or not all(math.isfinite(v) and v > 0. for v in size):
        raise ValueError('BUNKER box dimensions must be finite and positive')
    origin = collision.find('origin')
    xyz = [float(v) for v in (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()]
    rpy = [float(v) for v in (origin.get('rpy', '0 0 0') if origin is not None else '0 0 0').split()]
    if len(xyz) != 3 or len(rpy) != 3 or not all(math.isfinite(v) for v in xyz+rpy):
        raise ValueError('BUNKER collision origin must be finite')
    children = {j.find('child').get('link') for j in root.findall('joint')}
    roots = {link.get('name') for link in root.findall('link')} - children
    if roots != {'ground/base_link'}:
        raise ValueError('clearance copy requires fixed-root ground/base_link convention')
    rigid = {'ground/base_link'}
    while True:
        enlarged = rigid | {j.find('child').get('link') for j in root.findall('joint')
                           if j.get('type') == 'fixed' and j.find('parent').get('link') in rigid}
        if enlarged == rigid:
            break
        rigid = enlarged
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = xyz
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion_from_euler(*rpy)
    return size, pose, rigid


def chassis_clearance_diff(scene, robot_description, robot_link_names):
    size, pose, rigid = chassis_geometry(robot_description)
    change = ms._diff()
    guard = CollisionObject(id=CHASSIS_GUARD_ID, operation=CollisionObject.ADD)
    guard.header.frame_id = 'ground/base_link'
    guard.pose.orientation.w = 1.
    guard.primitives = [SolidPrimitive(type=SolidPrimitive.BOX,
                                       dimensions=[v + 2*CHASSIS_MARGIN_M for v in size])]
    guard.primitive_poses = [pose]
    change.world.collision_objects = [guard]
    # Explicitly include the future payload so later target/contact merges keep
    # this pair blocked; a world-world overlap alone is not a payload check.
    temporary = copy.deepcopy(scene)
    if not any(o.id == ms.TARGET_ID for o in temporary.world.collision_objects):
        temporary.world.collision_objects.append(CollisionObject(id=ms.TARGET_ID))
    acm = ms._target_acm(temporary, robot_link_names, CHASSIS_GUARD_ID, False)
    index = acm.entry_names.index(CHASSIS_GUARD_ID)
    for other, name in enumerate(acm.entry_names):
        allowed = name in rigid
        acm.entry_values[index].enabled[other] = allowed
        acm.entry_values[other].enabled[index] = allowed
    change.allowed_collision_matrix = acm
    return change


def closure_positions(start, contact, spatial_step):
    end = max(contact, OBSERVED_LOADED_JOINT_MAX)
    if not all(math.isfinite(v) for v in (start, contact, spatial_step)) or spatial_step <= 0.:
        raise ValueError('closure sampling must be finite with a positive spatial step')
    if not 0. <= start <= .93 or not 0. <= contact <= .93:
        raise ValueError('closure sample outside AG95 stroke')
    if start > end:
        raise ValueError('measured closure already exceeds checked loaded range')
    intervals = max(1, int(math.ceil(abs(end-start)*.110/spatial_step)))
    return [start + (end-start)*i/intervals for i in range(intervals+1)]


def state_at_positions(start, names, positions):
    full_names, full_positions = start.joint_state.name, start.joint_state.position
    if (not full_names or len(full_names) != len(full_positions) or
            len(set(full_names)) != len(full_names) or any(not name for name in full_names) or
            not all(math.isfinite(v) for v in full_positions)):
        raise ValueError('complete start state must have unique matching finite joint values')
    if not names or len(names) != len(positions) or len(set(names)) != len(names):
        raise ValueError('trajectory joint positions must have unique matching names')
    result = copy.deepcopy(start)
    lookup = dict(zip(names, positions))
    if not set(names) <= set(result.joint_state.name) or not all(math.isfinite(v) for v in positions):
        raise ValueError('trajectory names/values disagree with complete robot state')
    result.joint_state.position = [lookup.get(name, value) for name, value in
                                   zip(result.joint_state.name, result.joint_state.position)]
    return result


def controller_samples(start, trajectory, period_s=CONTROLLER_SAMPLE_PERIOD_S, held_desired=None):
    """Sample the JTC linear/cubic/quintic polynomial, not linear MoveIt FK.

    Samples all knots with at most the SIM physics-step spacing, not an exact
    global time lattice or continuous collision certification. Runtime callers
    supply a fresh held desired point after the prior serial action terminates:
    JTC drops a zero-time first point and bridges from that held desired state.
    Without it this function checks only the nominal specified polynomial.
    """
    points = list(trajectory.joint_trajectory.points)
    names = trajectory.joint_trajectory.joint_names
    if not points or not math.isfinite(period_s) or period_s <= 0.:
        raise ValueError('nonempty timed trajectory and positive sample period required')
    if held_desired is not None:
        if trajectory.joint_trajectory.header.stamp.to_sec() != 0.:
            raise ValueError('held-desired bridge only models immediate JTC commands')
        n = len(names)
        if (len(held_desired.positions) != n or len(held_desired.velocities) != n or
                len(held_desired.accelerations) != n or
                not all(math.isfinite(v) for v in held_desired.positions) or
                any(v != 0. for v in list(held_desired.velocities)+list(held_desired.accelerations))):
            raise ValueError('finite stationary held desired state required')
        if points[0].time_from_start.to_sec() == 0.:
            points = points[1:]
        if not points:
            raise ValueError('JTC command has no future trajectory point')
        bridge = copy.deepcopy(held_desired)
        bridge.time_from_start = type(bridge.time_from_start)()
        points.insert(0, bridge)
    times = [p.time_from_start.to_sec() for p in points]
    if times[0] != 0. or any(not math.isfinite(t) for t in times) or any(
            b <= a for a, b in zip(times, times[1:])):
        raise ValueError('executed trajectory must start at zero and have increasing times')
    yield state_at_positions(start, names, points[0].positions)
    for first, second, t0, t1 in zip(points, points[1:], times, times[1:]):
        dt = t1-t0
        n = len(names)
        if len(first.positions) != n or len(second.positions) != n:
            raise ValueError('trajectory positions are incomplete')
        q0, q1 = np.asarray(first.positions), np.asarray(second.positions)
        derivatives = (first.velocities, second.velocities, first.accelerations, second.accelerations)
        if any(len(d) not in (0, n) for d in derivatives):
            raise ValueError('trajectory derivatives are incomplete')
        have_velocity = len(first.velocities) == n and len(second.velocities) == n
        have_acceleration = have_velocity and len(first.accelerations) == n and len(second.accelerations) == n
        if have_acceleration:
            v0, v1 = np.asarray(first.velocities)*dt, np.asarray(second.velocities)*dt
            a0, a1 = np.asarray(first.accelerations)*dt*dt, np.asarray(second.accelerations)*dt*dt
            c0, c1, c2 = q0, v0, a0/2
            rhs = np.array([q1-c0-c1-c2, v1-c1-2*c2, a1-2*c2])
            c3, c4, c5 = np.linalg.solve(np.array([[1., 1., 1.], [3., 4., 5.], [6., 12., 20.]]), rhs)
            coefficients = [c0, c1, c2, c3, c4, c5]
        elif have_velocity:
            v0, v1 = np.asarray(first.velocities)*dt, np.asarray(second.velocities)*dt
            coefficients = [q0, v0, 3*(q1-q0)-2*v0-v1, 2*(q0-q1)+v0+v1]
        else:
            coefficients = [q0, q1-q0]
        if not all(np.all(np.isfinite(c)) for c in coefficients):
            raise ValueError('trajectory polynomial must be finite')
        intervals = max(1, int(math.ceil(dt/period_s)))
        for i in range(1, intervals+1):
            s = i/intervals
            q = sum(c*s**power for power, c in enumerate(coefficients))
            # Exact endpoint avoids accumulated polynomial roundoff.
            if i == intervals:
                q = q1
            yield state_at_positions(start, names, q.tolist())
