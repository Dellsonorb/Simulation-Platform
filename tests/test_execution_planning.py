"""Offline full-message tests; no ROS master, planner, or controller is started."""
import copy
import importlib
import math
from pathlib import Path
import sys
import types
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/demos/air_ground_pick_demo/src'))
sys.path.insert(0, str(ROOT / 'tests'))
try:
    import rospy
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from moveit_msgs.msg import PlanningScene, RobotState, RobotTrajectory, CollisionObject
    from moveit_msgs.srv import GetPositionIKResponse, GetCartesianPathResponse, GetStateValidityResponse
    from trajectory_msgs.msg import JointTrajectoryPoint
    from control_msgs.msg import JointTrajectoryControllerState
    from actionlib_msgs.msg import GoalStatusArray
    from air_ground_pick_demo import execution_clearance as ec, manipulation_scene as ms
    from test_execution_clearance import URDF
    from test_full_robot_manipulation import demo, target_pose
    NATIVE = True
except ImportError:
    NATIVE = False


@unittest.skipUnless(NATIVE, 'native ROS messages required')
class ExecutionPlanningTests(unittest.TestCase):
    def setUp(self):
        spec = importlib.util.find_spec('air_ground_pick_demo.execution_planning')
        self.assertIsNotNone(spec, 'bounded execution planning module is missing')
        self.ep = importlib.import_module('air_ground_pick_demo.execution_planning')
        import yaml
        owner = demo.AirGroundPickDemo.__new__(demo.AirGroundPickDemo)
        config = yaml.safe_load((ROOT / 'src/demos/air_ground_pick_demo/config/demo.yaml').read_text())
        for name, value in config.items():
            setattr(owner, '_' + name, value)
        owner._execution_clearance = True
        owner._full_robot_manipulation = True
        owner._move_group_name = config['move_group']
        owner._scene_robot_links = ['ground/base_link', 'mount', 'arm', 'ground/d435',
                                    owner._end_effector_link] + list(ms.FINGER_LINKS)
        self.state = RobotState()
        self.state.joint_state.name = list(owner._observation_joint_names) + ['left_outer_knuckle_joint']
        self.state.joint_state.position = [0.] * 6 + [.25]
        self.scene = PlanningScene()
        self.scene.robot_state = copy.deepcopy(self.state)
        self.scene.world.collision_objects = [CollisionObject(id='floor')]
        self.applied, self.requests, self.actions, self.statuses = [], [], [], []
        self.reject = None
        self.owner = owner
        owner._get_manipulation_scene = lambda: copy.deepcopy(self.scene)
        owner._robot_state_from_joint_feedback = lambda: self.measured()
        owner._publish_status = lambda state, **values: self.statuses.append((state, values))
        owner._apply_manipulation_scene = self.apply
        owner._state_validity = self.validity
        owner._check_full_robot_state = lambda label, state=None, require_payload=False: self.ep.check_state(
            owner, self.measured() if state is None else state, label, require_payload)
        owner._execution_ik = self.ik
        owner._cartesian_path = self.cartesian
        owner._move_group = types.SimpleNamespace(
            get_planning_frame=lambda: 'ground/base_link',
            get_active_joints=lambda: list(owner._observation_joint_names),
            set_start_state=lambda state: None,
            set_joint_value_target=lambda q: None,
            clear_pose_targets=lambda: None,
            plan=lambda: (True, self.trajectory(self.state, [.2] * 6)),
            retime_trajectory=lambda state, trajectory, *scales: copy.deepcopy(trajectory),
            execute=lambda *args, **kw: self.actions.append('FORBIDDEN_group_execute'))
        owner._initialize_moveit = lambda: owner._move_group
        self.now = rospy.Time.from_sec(10.)
        transform = TransformStamped()
        transform.header.frame_id = 'map'
        transform.header.stamp = self.now
        transform.transform.rotation.w = 1.
        transform.transform.translation.z = .412
        owner._tf_buffer = types.SimpleNamespace(lookup_transform=lambda *args: copy.deepcopy(transform))
        self.patches = [mock.patch.object(rospy.Time, 'now', return_value=self.now),
                        mock.patch.object(rospy, 'get_param', return_value=URDF),
                        mock.patch.object(rospy, 'is_shutdown', return_value=False)]
        for patch in self.patches:
            patch.start()
            self.addCleanup(patch.stop)

    def measured(self):
        state = copy.deepcopy(self.state)
        state.attached_collision_objects = copy.deepcopy(self.scene.robot_state.attached_collision_objects)
        return state

    def apply(self, change):
        self.applied.append(copy.deepcopy(change))
        if not change.is_diff:
            self.scene = copy.deepcopy(change)
            return
        if change.allowed_collision_matrix.entry_names:
            self.scene.allowed_collision_matrix = copy.deepcopy(change.allowed_collision_matrix)
        for obj in change.world.collision_objects:
            self.scene.world.collision_objects = [o for o in self.scene.world.collision_objects if o.id != obj.id]
            if obj.operation != CollisionObject.REMOVE:
                self.scene.world.collision_objects.append(copy.deepcopy(obj))
        for item in change.robot_state.attached_collision_objects:
            self.scene.robot_state.attached_collision_objects = [a for a in self.scene.robot_state.attached_collision_objects
                                                                  if a.object.id != item.object.id]
            if item.object.operation != CollisionObject.REMOVE:
                self.scene.world.collision_objects = [o for o in self.scene.world.collision_objects if o.id != item.object.id]
                self.scene.robot_state.attached_collision_objects.append(copy.deepcopy(item))

    def validity(self, request):
        self.requests.append(copy.deepcopy(request))
        self.assertEqual('', request.group_name)
        acm = self.scene.allowed_collision_matrix
        if ec.CHASSIS_GUARD_ID in acm.entry_names and ms.TARGET_ID in acm.entry_names:
            self.assertFalse(acm.entry_values[acm.entry_names.index(ec.CHASSIS_GUARD_ID)].enabled[
                acm.entry_names.index(ms.TARGET_ID)])
        q = dict(zip(request.robot_state.joint_state.name, request.robot_state.joint_state.position))
        payload = bool(request.robot_state.attached_collision_objects)
        invalid = (self.reject == 'closure' and q['left_outer_knuckle_joint'] > .50 or
                   self.reject == 'lift' and payload)
        return GetStateValidityResponse(valid=not invalid)

    def trajectory(self, state, goal):
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = list(self.owner._observation_joint_names)
        q = dict(zip(state.joint_state.name, state.joint_state.position))
        trajectory.joint_trajectory.points = [
            JointTrajectoryPoint(positions=[q[n] for n in trajectory.joint_trajectory.joint_names],
                                 velocities=[0.] * 6, accelerations=[0.] * 6,
                                 time_from_start=rospy.Duration(0.)),
            JointTrajectoryPoint(positions=goal, velocities=[0.] * 6, accelerations=[0.] * 6,
                                 time_from_start=rospy.Duration(.004))]
        return trajectory

    def ik(self, request):
        self.assertTrue(request.ik_request.avoid_collisions)
        response = GetPositionIKResponse()
        response.error_code.val = 1
        response.solution = ec.state_at_positions(request.ik_request.robot_state,
                                                 self.owner._observation_joint_names, [.3] * 6)
        return response

    def cartesian(self, request):
        self.assertTrue(request.avoid_collisions)
        response = GetCartesianPathResponse()
        response.error_code.val, response.fraction = 1, 1.
        response.solution = self.trajectory(request.start_state, [.2] * 6)
        return response

    def install_target(self):
        pose = target_pose(.7, .1, -.35, .2)
        pose.header.frame_id = 'ground/base_link'
        self.apply(ms.world_target_diff(self.scene, pose, self.owner._target_size, self.owner._scene_robot_links))
        self.ep.install_guard(self.owner)
        return pose

    def test_preview_reject_restores_exact_scene_and_never_executes(self):
        before = copy.deepcopy(self.scene)
        self.reject = 'lift'
        result = self.owner._preview_ground_candidate((2., .1, .0575, .2),
                                                      dict(x=1., y=.3, yaw=.4), None)
        self.assertFalse(result['feasible'])
        self.assertEqual(6, len(result['attempts']))
        self.assertEqual(before, self.scene)
        self.assertEqual([], self.actions)
        self.assertTrue(all(row['stage'] == 'attached_lift' for row in result['attempts']))

    def test_preview_uses_measured_base_z_and_does_not_change_candidate(self):
        candidate = dict(x=1., y=.3, yaw=.4)
        result = self.owner._preview_ground_candidate((2., .1, .0575, .2), candidate, [0.] * 6)
        self.assertTrue(result['feasible'])
        self.assertAlmostEqual(.0575 - .412, result['target_base'][2])
        self.assertEqual(dict(x=1., y=.3, yaw=.4), candidate)
        self.assertEqual('aerial_predicted', result['source'])
        self.assertTrue(result['arrival_revalidation_required'])

    def test_service_exception_restores_scene_and_propagates(self):
        before = copy.deepcopy(self.scene)
        self.owner._execution_ik = mock.Mock(side_effect=rospy.ServiceException('transport gone'))
        with self.assertRaisesRegex(rospy.ServiceException, 'transport gone'):
            self.owner._preview_ground_candidate((2., .1, .0575, .2), dict(x=1., y=0., yaw=0.), None)
        self.assertEqual(before, self.scene)

    def test_seed_schedule_has_three_fixed_named_seeds_for_each_symmetry(self):
        rows = self.ep.branch_seeds(self.state, list(reversed(self.owner._observation_joint_names)), [.4] * 6)
        self.assertEqual([0., 0., 0., math.pi, math.pi, math.pi], [r[0] for r in rows])
        self.assertEqual(['measured', 'rm_or_home', 'alternate'] * 2, [r[1] for r in rows])
        for _, name, state in rows:
            if name == 'rm_or_home':
                self.assertEqual([.4] * 6, state.joint_state.position[:6])

    def test_closure_failure_prevents_actual_approach_and_restores_contacts(self):
        pose = self.install_target()
        before = copy.deepcopy(self.scene)
        self.reject = 'closure'
        result = self.ep.search(self.owner, (.7, .1, -.35, .2), 'ground/base_link', pose.header.stamp,
                                None, actual=True, held_desired=JointTrajectoryPoint(
                                    positions=[0.] * 6, velocities=[0.] * 6, accelerations=[0.] * 6))
        self.assertFalse(result['feasible'])
        self.assertEqual(6, len(result['attempts']))
        self.assertEqual(before, self.scene)
        self.assertEqual([], self.actions)

    def test_checked_dispatch_samples_desired_bridge_before_single_serial_send(self):
        self.install_target()
        desired = JointTrajectoryControllerState()
        desired.header.stamp = self.now
        desired.joint_names = list(reversed(self.owner._observation_joint_names))
        desired.desired = JointTrajectoryPoint(positions=[.01] * 6, velocities=[0.] * 6, accelerations=[0.] * 6)
        action = types.SimpleNamespace(wait_for_server=lambda timeout: True,
            send_goal=lambda goal: self.actions.append(copy.deepcopy(goal)),
            wait_for_result=lambda timeout: True, get_state=lambda: 3, gh=None)
        self.owner._arm_client = action
        def message(topic, message_type, timeout):
            if message_type is GoalStatusArray:
                return GoalStatusArray()
            return desired
        trajectory = self.trajectory(self.state, [.2] * 6)
        with mock.patch.object(rospy, 'wait_for_message', side_effect=message), \
                mock.patch.object(ec, 'controller_samples', wraps=ec.controller_samples) as samples:
            self.owner._execute_checked_arm(trajectory, 'test arm')
        self.assertEqual(1, len(self.actions))
        self.assertEqual([.01] * 6, samples.call_args.kwargs['held_desired'].positions)
        self.assertGreater(len(self.requests), len(trajectory.joint_trajectory.points))

    def test_active_controller_owner_blocks_checked_dispatch(self):
        from actionlib_msgs.msg import GoalStatus
        self.install_target()
        self.owner._arm_client = types.SimpleNamespace(gh=None)
        status = GoalStatusArray(status_list=[GoalStatus(status=GoalStatus.ACTIVE)])
        with mock.patch.object(rospy, 'wait_for_message', return_value=status):
            with self.assertRaisesRegex(Exception, 'active|pending|ownership'):
                self.owner._execute_checked_arm(self.trajectory(self.state, [.2] * 6), 'blocked')
        self.assertEqual([], self.actions)

    def test_preview_waits_for_fresh_public_tf_within_existing_budget(self):
        clock = [100.]
        stale = self.owner._tf_buffer.lookup_transform()
        stale.header.stamp = rospy.Time.from_sec(1.)
        fresh = self.owner._tf_buffer.lookup_transform()
        self.owner._tf_buffer.lookup_transform = mock.Mock(side_effect=[stale, fresh])
        with mock.patch.object(self.ep.time, 'monotonic', side_effect=lambda: clock[0]), \
                mock.patch.object(rospy.rostime, 'wallsleep', side_effect=lambda dt: clock.__setitem__(0, clock[0]+dt)):
            result = self.owner._preview_ground_candidate((2., .1, .0575, .2), dict(x=1., y=0., yaw=0.), None)
        self.assertTrue(result['feasible'])
        self.assertEqual(2, self.owner._tf_buffer.lookup_transform.call_count)

    def test_actual_complete_search_restores_scene_before_pregrasp_and_keeps_original_stages(self):
        self.install_target()
        self.owner._map_frame = 'ground/base_link'
        original = copy.deepcopy(self.scene)
        self.owner._update_manipulation_target = lambda *args, **kw: None
        self.owner._preshape_gripper = lambda: self.actions.append('preshape')
        self.owner._close_gripper = lambda: self.actions.append('real_close')
        self.owner._hold_grasp_confirmation = lambda: self.actions.append('retention')
        self.owner._execute_pregrasp = lambda *args: (
            self.assertEqual(original, self.scene), self.actions.append('pregrasp'))
        self.owner._execute_cartesian = lambda target, label: (
            self.actions.append(label), target)[1]
        held = JointTrajectoryPoint(positions=[0.] * 6, velocities=[0.] * 6, accelerations=[0.] * 6)
        with mock.patch.object(self.ep, 'stationary_hold', return_value=(self.state, held)):
            self.owner._pick_and_lift(target_pose(), (.7, .1, -.35, .2))
        self.assertEqual(['preshape', 'pregrasp', 'grasp approach', 'real_close', 'lift', 'retention'], self.actions)
        self.assertIsNone(self.owner._execution_selected)

    def test_lift_collision_blocks_pick_before_pregrasp_wrapper(self):
        self.install_target()
        self.owner._map_frame = 'ground/base_link'
        self.owner._update_manipulation_target = lambda *args, **kw: None
        self.owner._preshape_gripper = lambda: None
        self.owner._execute_pregrasp = lambda *args: self.actions.append('pregrasp')
        self.reject = 'lift'
        held = JointTrajectoryPoint(positions=[0.] * 6, velocities=[0.] * 6, accelerations=[0.] * 6)
        with mock.patch.object(self.ep, 'stationary_hold', return_value=(self.state, held)):
            with self.assertRaisesRegex(demo.DemoError, 'six branches'):
                self.owner._pick_and_lift(target_pose(), (.7, .1, -.35, .2))
        self.assertEqual([], self.actions)
        self.assertIsNone(self.owner._execution_selected)

    def test_no_nominal_loaded_endpoint_omission_when_closure_grid_skips_it(self):
        self.install_target()
        result = self.ep.search(self.owner, (.7, .1, -.35, .2), 'ground/base_link', self.now, None)
        self.assertTrue(result['feasible'])
        loaded = [dict(zip(r.robot_state.joint_state.name, r.robot_state.joint_state.position))[
                    'left_outer_knuckle_joint'] for r in self.requests if r.robot_state.attached_collision_objects]
        geometry = ms.ag95_contact_geometry(self.owner._target_size, self.owner._maximum_gripper_opening,
            self.owner._maximum_gripper_joint, self.owner._finger_pad_lower_edge_offset)
        self.assertIn(geometry.q_contact, loaded)
        self.assertIn(.522, loaded)

    def test_controller_rejects_nonstationary_desired_without_sending(self):
        self.install_target()
        self.owner._arm_client = types.SimpleNamespace(gh=None)
        state = JointTrajectoryControllerState()
        state.header.stamp = self.now
        state.joint_names = list(self.owner._observation_joint_names)
        state.desired = JointTrajectoryPoint(positions=[0.] * 6, velocities=[1e-12] * 6, accelerations=[0.] * 6)
        clock = [100.]
        def read(topic, kind, timeout):
            if kind is GoalStatusArray:
                return GoalStatusArray()
            clock[0] += self.owner._arm_action_timeout
            return state
        with mock.patch.object(self.ep.time, 'monotonic', side_effect=lambda: clock[0]), \
                mock.patch.object(rospy, 'wait_for_message', side_effect=read):
            with self.assertRaisesRegex(Exception, 'hold timeout'):
                self.owner._execute_checked_arm(self.trajectory(self.state, [.2] * 6), 'not held')
        self.assertEqual([], self.actions)

    def test_trajectory_timeout_is_interface_error_not_success_or_next_branch(self):
        self.install_target()
        import threading
        gate = threading.Event()
        self.owner._state_validity = lambda request: (gate.wait(2.), GetStateValidityResponse(valid=True))[1]
        try:
            with mock.patch.object(self.ep, 'CHECK_WALL_TIMEOUT_S', .01):
                with self.assertRaisesRegex(RuntimeError, 'wall budget'):
                    self.ep.check_samples(self.owner, [self.state], 'stalled validity')
        finally:
            gate.set()

    def test_loaded_range_exceeded_blocks_fresh_lift_before_arm_execution(self):
        self.install_target()
        self.state.joint_state.position[-1] = .523
        self.owner._check_full_robot_state = lambda *args, **kwargs: copy.deepcopy(self.state)
        self.owner._execute_checked_arm = lambda *args: self.actions.append('arm')
        with self.assertRaisesRegex(demo.DemoError, 'checked loaded range'):
            self.owner._execute_cartesian(target_pose(), 'lift')
        self.assertEqual([], self.actions)

    def test_mode_off_pick_keeps_original_sequence_without_clearance_search(self):
        self.owner._execution_clearance = False
        self.owner._pick_and_lift_sequence = mock.Mock(return_value='original')
        self.owner._pick_and_lift_guarded = mock.Mock(side_effect=AssertionError('guard called in mode off'))
        self.assertEqual('original', self.owner._pick_and_lift(target_pose(), (2., .1, .0575, .2)))

    def test_controller_status_wait_counts_in_held_state_wall_budget(self):
        self.install_target()
        self.owner._arm_client = types.SimpleNamespace(gh=None)
        clock = [100.]
        def status_only(*args, **kwargs):
            clock[0] += self.owner._arm_action_timeout + .01
            return GoalStatusArray()
        with mock.patch.object(self.ep.time, 'monotonic', side_effect=lambda: clock[0]), \
                mock.patch.object(rospy, 'wait_for_message', side_effect=status_only):
            with self.assertRaisesRegex(RuntimeError, 'hold timeout'):
                self.ep.stationary_hold(self.owner, self.owner._observation_joint_names)

    def test_controller_success_with_invalid_measured_endpoint_still_fails(self):
        self.install_target()
        held = JointTrajectoryPoint(positions=[0.] * 6, velocities=[0.] * 6, accelerations=[0.] * 6)
        owner = self.owner
        def completed(timeout):
            self.reject = 'closure'
            self.state.joint_state.position[-1] = .51
            return True
        owner._arm_client = types.SimpleNamespace(wait_for_server=lambda timeout: True,
            send_goal=lambda goal: self.actions.append('arm'), wait_for_result=completed, get_state=lambda: 3)
        with mock.patch.object(self.ep, 'stationary_hold', return_value=(self.state, held)):
            with self.assertRaisesRegex(demo.DemoError, 'collision'):
                owner._execute_checked_arm(self.trajectory(self.state, [.2] * 6), 'endpoint')
        self.assertTrue(owner._execution_arm_failed)

    def test_arm_abort_prevents_any_further_dispatch(self):
        self.install_target()
        held = JointTrajectoryPoint(positions=[0.] * 6, velocities=[0.] * 6, accelerations=[0.] * 6)
        owner = self.owner
        owner._arm_client = types.SimpleNamespace(wait_for_server=lambda timeout: True,
            send_goal=lambda goal: self.actions.append('arm'), wait_for_result=lambda timeout: True,
            get_state=lambda: 4)
        with mock.patch.object(self.ep, 'stationary_hold', return_value=(self.state, held)):
            with self.assertRaisesRegex(demo.DemoError, 'action failed'):
                owner._execute_checked_arm(self.trajectory(self.state, [.2] * 6), 'aborted')
        with self.assertRaisesRegex(demo.DemoError, 'ownership unavailable'):
            owner._execute_checked_arm(self.trajectory(self.state, [.2] * 6), 'after abort')
        self.assertEqual(['arm'], self.actions)

    def test_runtime_lift_rechecks_loaded_range_on_fresh_replanned_curve(self):
        pose = self.install_target()
        self.apply(ec.predicted_payload_diff(self.scene, pose, self.owner._end_effector_link,
                                             self.owner._scene_robot_links))
        self.state.joint_state.position[-1] = .46
        observed = []
        original = self.validity
        def check(request):
            q = dict(zip(request.robot_state.joint_state.name, request.robot_state.joint_state.position))
            observed.append(q['left_outer_knuckle_joint'])
            if q['left_outer_knuckle_joint'] > .51:
                return GetStateValidityResponse(valid=False)
            return original(request)
        self.owner._state_validity = check
        self.owner._verify_tcp_pose = lambda target, label: target
        held = JointTrajectoryPoint(positions=[.01] * 6, velocities=[0.] * 6, accelerations=[0.] * 6)
        with mock.patch.object(self.ep, 'stationary_hold', side_effect=lambda *args: (self.measured(), held)):
            with self.assertRaisesRegex(Exception, 'collision'):
                self.owner._execute_cartesian(pose, 'lift')
        self.assertEqual([], self.actions)
        self.assertGreater(max(observed), .51)

    def test_loaded_runtime_samples_real_held_bridge_at_every_articulation(self):
        pose = self.install_target()
        self.apply(ec.predicted_payload_diff(self.scene, pose, self.owner._end_effector_link,
                                             self.owner._scene_robot_links))
        self.state.joint_state.position[-1] = .46
        held = JointTrajectoryPoint(positions=[.01] * 6, velocities=[0.] * 6, accelerations=[0.] * 6)
        self.owner._arm_client = types.SimpleNamespace(wait_for_server=lambda timeout: True,
            send_goal=lambda goal: self.actions.append('lift'), wait_for_result=lambda timeout: True,
            get_state=lambda: 3)
        with mock.patch.object(self.ep, 'stationary_hold', side_effect=lambda *args: (self.measured(), held)), \
                mock.patch.object(ec, 'controller_samples', wraps=ec.controller_samples) as sampling:
            self.owner._execute_checked_arm(self.trajectory(self.state, [.2] * 6), 'lift')
        articulations = [dict(zip(call.args[0].joint_state.name, call.args[0].joint_state.position))[
            'left_outer_knuckle_joint'] for call in sampling.call_args_list if call.kwargs.get('held_desired') is not None]
        self.assertIn(.522, articulations)
        geometry = ms.ag95_contact_geometry(self.owner._target_size, self.owner._maximum_gripper_opening,
            self.owner._maximum_gripper_joint, self.owner._finger_pad_lower_edge_offset)
        self.assertIn(geometry.q_contact, articulations)

    def test_camera_opt_in_checks_full_start_and_uses_shared_dispatch(self):
        import threading
        from sensor_msgs.msg import CameraInfo
        from air_ground_pick_demo.ground_observation import observe_from_camera_poses
        self.install_target()
        owner = self.owner
        owner._lock, owner._ground_surface_cue = threading.RLock(), None
        owner._open_gripper = lambda: .09
        owner._wait_for_ground_target = mock.Mock(side_effect=[demo.DemoError('need camera view'),
                                                              (target_pose(), (.7, .1, .0575, .2))])
        owner._execute_checked_arm = lambda *args: self.actions.append('checked_arm')
        owner._verify_tcp_pose = lambda *args: None
        owner._move_group.set_pose_target = lambda *args: None
        owner._move_group.set_start_state_to_current_state = mock.Mock(side_effect=AssertionError('unchecked start'))
        info = CameraInfo(width=640, height=480, K=[400., 0., 320., 0., 400., 240., 0., 0., 1.])
        info.header.frame_id = 'ground/camera'
        with mock.patch.object(rospy, 'wait_for_message', return_value=info):
            observe_from_camera_poses(owner, (.7, .1, .0575, .2), demo.DemoError)
        self.assertEqual(['checked_arm'], self.actions)
        self.assertTrue(self.requests)
        self.assertEqual('', self.requests[0].group_name)

    def test_camera_invalid_actual_start_prevents_plan_and_dispatch(self):
        import threading
        from sensor_msgs.msg import CameraInfo
        from air_ground_pick_demo.ground_observation import observe_from_camera_poses
        self.install_target()
        owner = self.owner
        owner._lock, owner._ground_surface_cue = threading.RLock(), None
        owner._open_gripper = lambda: .09
        owner._wait_for_ground_target = mock.Mock(side_effect=demo.DemoError('need camera view'))
        owner._execute_checked_arm = lambda *args: self.actions.append('checked_arm')
        owner._move_group.plan = mock.Mock(side_effect=AssertionError('planned invalid start'))
        owner._move_group.set_pose_target = lambda *args: None
        self.reject, self.state.joint_state.position[-1] = 'closure', .51
        info = CameraInfo(width=640, height=480, K=[400., 0., 320., 0., 400., 240., 0., 0., 1.])
        info.header.frame_id = 'ground/camera'
        with mock.patch.object(rospy, 'wait_for_message', return_value=info):
            with self.assertRaisesRegex(Exception, 'collision'):
                observe_from_camera_poses(owner, (.7, .1, .0575, .2), demo.DemoError)
        self.assertEqual([], self.actions)

    def test_loaded_cap_is_enforced_at_both_fresh_controller_acquisitions(self):
        for first_q, last_q in ((.523, .523), (.46, .523)):
            with self.subTest(first=first_q, last=last_q):
                self.scene.robot_state.attached_collision_objects = []
                pose = self.install_target()
                self.apply(ec.predicted_payload_diff(self.scene, pose, self.owner._end_effector_link,
                                                     self.owner._scene_robot_links))
                self.owner._execution_arm_failed = False
                self.owner._arm_client = types.SimpleNamespace(wait_for_server=lambda timeout: True,
                    send_goal=lambda goal: self.actions.append('UNSAFE_SEND'), wait_for_result=lambda timeout: True,
                    get_state=lambda: 3)
                held = JointTrajectoryPoint(positions=[.01] * 6, velocities=[0.] * 6, accelerations=[0.] * 6)
                states = [ec.state_at_positions(self.measured(), ['left_outer_knuckle_joint'], [q])
                          for q in (first_q, last_q)]
                with mock.patch.object(self.ep, 'stationary_hold', side_effect=[(state, held) for state in states]):
                    with self.assertRaisesRegex(demo.DemoError, 'checked loaded range'):
                        self.owner._execute_checked_arm(self.trajectory(self.state, [.2] * 6), 'lift')
                self.assertEqual([], self.actions)

    def test_second_fresh_bridge_is_checked_at_loaded_max_before_send(self):
        pose = self.install_target()
        self.apply(ec.predicted_payload_diff(self.scene, pose, self.owner._end_effector_link,
                                             self.owner._scene_robot_links))
        self.state.joint_state.position[-1] = .46
        held = JointTrajectoryPoint(positions=[.01] * 6, velocities=[0.] * 6, accelerations=[0.] * 6)
        count = [0]
        def acquire(*args):
            count[0] += 1
            return self.measured(), held
        original = self.validity
        def validity(request):
            q = dict(zip(request.robot_state.joint_state.name, request.robot_state.joint_state.position))
            if count[0] == 2 and q['left_outer_knuckle_joint'] > .51:
                return GetStateValidityResponse(valid=False)
            return original(request)
        self.owner._state_validity = validity
        self.owner._arm_client = types.SimpleNamespace(wait_for_server=lambda timeout: True,
            send_goal=lambda goal: self.actions.append('UNSAFE_SEND'), wait_for_result=lambda timeout: True,
            get_state=lambda: 3)
        with mock.patch.object(self.ep, 'stationary_hold', side_effect=acquire):
            with self.assertRaisesRegex(demo.DemoError, 'collision'):
                self.owner._execute_checked_arm(self.trajectory(self.state, [.2] * 6), 'lift')
        self.assertEqual([], self.actions)

    def test_ik_semantic_interface_failure_propagates_without_six_rejections(self):
        response = GetPositionIKResponse()
        response.error_code.val = -18  # INVALID_LINK_NAME
        self.owner._execution_ik = mock.Mock(return_value=response)
        before = copy.deepcopy(self.scene)
        with self.assertRaisesRegex(RuntimeError, 'interface|configuration'):
            self.owner._preview_ground_candidate((2., .1, .0575, .2), dict(x=1., y=0., yaw=0.), None)
        self.assertEqual(1, self.owner._execution_ik.call_count)
        self.assertEqual(before, self.scene)

    def test_cartesian_semantic_interface_failure_propagates_and_restores_scene(self):
        response = GetCartesianPathResponse()
        response.error_code.val = -21  # FRAME_TRANSFORM_FAILURE
        self.owner._cartesian_path = mock.Mock(return_value=response)
        before = copy.deepcopy(self.scene)
        with self.assertRaisesRegex(RuntimeError, 'interface|configuration'):
            self.owner._preview_ground_candidate((2., .1, .0575, .2), dict(x=1., y=0., yaw=0.), None)
        self.assertEqual(1, self.owner._cartesian_path.call_count)
        self.assertEqual(before, self.scene)

    def test_move_group_semantic_interface_failure_is_not_candidate_infeasibility(self):
        from moveit_msgs.msg import MoveItErrorCodes
        self.install_target()
        self.owner._move_group.plan = mock.Mock(return_value=(False, RobotTrajectory(), .01, MoveItErrorCodes(val=-15)))
        with self.assertRaisesRegex(RuntimeError, 'interface|configuration'):
            self.ep.search(self.owner, (.7, .1, -.35, .2), 'ground/base_link', self.now,
                           None, actual=True, held_desired=JointTrajectoryPoint(
                               positions=[0.] * 6, velocities=[0.] * 6, accelerations=[0.] * 6))
        self.assertEqual(1, self.owner._move_group.plan.call_count)

    def test_no_ik_solution_is_still_an_ordinary_six_branch_rejection(self):
        response = GetPositionIKResponse()
        response.error_code.val = -31  # NO_IK_SOLUTION
        self.owner._execution_ik = mock.Mock(return_value=response)
        result = self.owner._preview_ground_candidate((2., .1, .0575, .2), dict(x=1., y=0., yaw=0.), None)
        self.assertFalse(result['feasible'])
        self.assertEqual(6, self.owner._execution_ik.call_count)

    def test_search_resolves_both_yaws_before_long_first_ik(self):
        self.install_target()
        expired = [False]
        def transform(pose, frame):
            if expired[0]:
                raise RuntimeError('old observation timestamp left TF cache')
            value = copy.deepcopy(pose)
            value.header.frame_id = frame
            return value
        def no_ik(request):
            expired[0] = True
            response = GetPositionIKResponse()
            response.error_code.val = -31
            return response
        self.owner._transform_pose = transform
        self.owner._execution_ik = no_ik
        result = self.ep.search(self.owner, (.7, .1, -.35, .2), 'map', self.now, None)
        self.assertFalse(result['feasible'])
        self.assertEqual(6, len(result['attempts']))

    def test_actual_pose_transforms_finish_before_preshape_and_are_not_repeated_after_search(self):
        self.install_target()
        expired = [False]
        def transform(pose, frame):
            if expired[0]:
                raise RuntimeError('old observation timestamp left TF cache')
            value = copy.deepcopy(pose)
            value.header.frame_id = frame
            return value
        self.owner._transform_pose = mock.Mock(side_effect=transform)
        self.owner._update_manipulation_target = lambda *args, **kwargs: None
        self.owner._preshape_gripper = lambda: expired.__setitem__(0, True)
        self.owner._execute_pregrasp = lambda *args: None
        self.owner._execute_cartesian = lambda pose, label: pose
        self.owner._close_gripper = lambda: None
        self.owner._hold_grasp_confirmation = lambda: None
        held = JointTrajectoryPoint(positions=[0.] * 6, velocities=[0.] * 6, accelerations=[0.] * 6)
        with mock.patch.object(self.ep, 'stationary_hold', return_value=(self.state, held)):
            self.owner._pick_and_lift(target_pose(), (.7, .1, -.35, .2))
        self.assertEqual(6, self.owner._transform_pose.call_count)

    def test_native_moveit_full_scene_restore_removes_hypothesis_and_preserves_original_acm(self):
        import io
        import shlex
        import struct
        import subprocess
        import tempfile
        from test_manipulation_scene import NATIVE_SCENE
        code = NATIVE_SCENE.split('int main(')[0] + r'''
int main() {
  auto robot = urdf::parseURDF("<robot name='probe'><link name='ground/base_link'/><link name='ground/gripper_tcp_link'/><joint name='tcp' type='fixed'><parent link='ground/base_link'/><child link='ground/gripper_tcp_link'/></joint></robot>");
  auto semantic = std::make_shared<srdf::Model>(); semantic->initString(*robot, "<robot name='probe'/>");
  planning_scene::PlanningScene scene(robot, semantic);
  auto original = readMessage();
  collision_detection::AllowedCollisionMatrix original_acm(original.allowed_collision_matrix);
  bool first = scene.setPlanningSceneMsg(original);
  collision_detection::AllowedCollision::Type initial_contact;
  bool initially_found = scene.getAllowedCollisionMatrix().getAllowedCollision("ground/base_link", "perceived_pick_target", initial_contact);
  bool guard = scene.setPlanningSceneDiffMsg(readMessage());
  bool payload = scene.setPlanningSceneDiffMsg(readMessage());
  bool has_payload = scene.getCurrentState().hasAttachedBody("perceived_pick_target");
  bool removed = scene.setPlanningSceneDiffMsg(readMessage());
  bool restored = scene.setPlanningSceneMsg(readMessage());
  collision_detection::AllowedCollision::Type contact = collision_detection::AllowedCollision::ALWAYS;
  bool found = scene.getAllowedCollisionMatrix().getAllowedCollision("ground/base_link", "perceived_pick_target", contact);
  bool same_acm = true;
  for (const auto& a : original.allowed_collision_matrix.entry_names) for (const auto& b : original.allowed_collision_matrix.entry_names) {
    collision_detection::AllowedCollision::Type expected = collision_detection::AllowedCollision::NEVER;
    collision_detection::AllowedCollision::Type actual = collision_detection::AllowedCollision::NEVER;
    original_acm.getAllowedCollision(a, b, expected);
    scene.getAllowedCollisionMatrix().getAllowedCollision(a, b, actual);
    if (expected != actual) same_acm = false;
  }
  std::cout << "RESTORED " << first << guard << payload << has_payload << removed << restored
            << " TARGET " << scene.getWorld()->hasObject("perceived_pick_target")
            << " GUARD " << scene.getWorld()->hasObject("execution_chassis_clearance")
            << " PAYLOAD " << scene.getCurrentState().hasAttachedBody("perceived_pick_target")
            << " ORIGINAL " << scene.getCurrentState().hasAttachedBody("original_payload")
            << " CONTACT " << (contact == collision_detection::AllowedCollision::NEVER)
            << " ACM_SAME " << same_acm << "\n";
}
'''
        accepted = target_pose(.7, .1, -.35, .2)
        accepted.header.frame_id = 'ground/base_link'
        self.scene = PlanningScene()
        self.apply(ms.world_target_diff(self.scene, accepted, self.owner._target_size, self.owner._scene_robot_links))
        from moveit_msgs.msg import AttachedCollisionObject
        original_payload = AttachedCollisionObject(link_name=self.owner._end_effector_link)
        original_payload.object = copy.deepcopy(self.scene.world.collision_objects[0])
        original_payload.object.id = 'original_payload'
        original_payload.object.header.frame_id = self.owner._end_effector_link
        self.scene.robot_state.attached_collision_objects = [original_payload]
        original = copy.deepcopy(self.scene)
        with self.ep.temporary_scene(self.owner):
            self.ep.install_guard(self.owner)
            guard = copy.deepcopy(self.applied[-1])
            predicted = ec.predicted_payload_diff(self.scene, accepted, self.owner._end_effector_link,
                                                  self.owner._scene_robot_links)
            self.apply(predicted)
        restored = self.applied[-1]
        removal = self.applied[-2]
        def packet(message):
            stream = io.BytesIO()
            message.serialize(stream)
            data = stream.getvalue()
            return struct.pack('<I', len(data)) + data
        flags = shlex.split(subprocess.check_output(['pkg-config', '--cflags', 'moveit_core'], text=True))
        libraries = shlex.split(subprocess.check_output(['pkg-config', '--libs', 'moveit_core'], text=True))
        with tempfile.TemporaryDirectory(prefix='p450-execution-restore-') as directory:
            executable = str(Path(directory) / 'restore')
            compiled = subprocess.run(['g++', '-std=c++14'] + flags + ['-x', 'c++', '-', '-x', 'none',
                                      '-o', executable] + libraries, input=code, text=True,
                                      capture_output=True, timeout=60)
            self.assertEqual(0, compiled.returncode, compiled.stderr)
            result = subprocess.run([executable], input=b''.join(map(packet, [original, guard, predicted, removal, restored])),
                                    capture_output=True, timeout=10)
            self.assertEqual(0, result.returncode, result.stderr)
            self.assertIn('RESTORED 111111 TARGET 1 GUARD 0 PAYLOAD 0 ORIGINAL 1 CONTACT 1', result.stdout.decode())
            self.assertIn('ACM_SAME 1', result.stdout.decode())
            self.scene = copy.deepcopy(original)
            self.ep.install_guard(self.owner)
            guarded_original = copy.deepcopy(self.scene)
            guard_again = copy.deepcopy(self.applied[-1])
            with self.ep.temporary_scene(self.owner):
                payload_again = ec.predicted_payload_diff(self.scene, accepted, self.owner._end_effector_link,
                                                          self.owner._scene_robot_links)
                self.apply(payload_again)
            messages = [guarded_original, guard_again, payload_again, self.applied[-2], self.applied[-1]]
            guarded = subprocess.run([executable], input=b''.join(map(packet, messages)),
                                     capture_output=True, timeout=10)
            self.assertEqual(0, guarded.returncode, guarded.stderr)
            self.assertIn('RESTORED 111111 TARGET 1 GUARD 1 PAYLOAD 0 ORIGINAL 1 CONTACT 1', guarded.stdout.decode())
            self.assertIn('ACM_SAME 1', guarded.stdout.decode())


if __name__ == '__main__':
    unittest.main()
