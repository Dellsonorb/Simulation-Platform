"""Offline runtime-hook tests using native ROS messages and fake transports."""

import copy
import importlib.util
import math
from pathlib import Path
import sys
import threading
import types
import unittest
from unittest import mock

import yaml

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src/demos/air_ground_pick_demo"
sys.path.insert(0, str(PACKAGE / "src"))
try:
    import rospy
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from moveit_msgs.msg import (AttachedCollisionObject, CollisionObject,
                                 PlanningScene, PlanningSceneComponents, RobotTrajectory)
    from moveit_msgs.srv import (ApplyPlanningSceneResponse, GetPlanningSceneResponse,
                                 GetStateValidityResponse)
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectoryPoint
    from air_ground_pick_demo import manipulation_scene as scene_helper
    spec = importlib.util.spec_from_file_location(
        "full_manipulation_demo", PACKAGE / "scripts/run_air_ground_pick_demo.py")
    demo = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(demo)
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


def target_pose(x=2., y=.1, z=.0575, yaw=.2):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = rospy.Time.from_sec(10.)
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
    pose.pose.orientation.z = math.sin(yaw/2.)
    pose.pose.orientation.w = math.cos(yaw/2.)
    return pose


@unittest.skipUnless(ROS_AVAILABLE, "native ROS message dependencies required")
class FullRobotManipulationTest(unittest.TestCase):
    def setUp(self):
        self.owner = demo.AirGroundPickDemo.__new__(demo.AirGroundPickDemo)
        owner = self.owner
        config = yaml.safe_load((PACKAGE / "config/demo.yaml").read_text())
        for key, value in config.items():
            setattr(owner, "_" + key, value)
        owner._full_robot_manipulation = True
        owner._placement_mode = "rm4d"
        owner._move_group_name = config["move_group"]
        owner._lock = threading.RLock()
        owner._ground_target_pose = None
        owner._ground_pose_received = None
        owner._ground_surface_cue = None
        owner._grasp_confirmed = False
        owner._grasp_confirmation_received = None
        owner._joint_state = JointState()
        owner._joint_state.header.stamp = rospy.Time.from_sec(10.)
        owner._joint_state.name = list(owner._observation_joint_names) + ["left_outer_knuckle_joint"]
        owner._joint_state.position = [0.] * 7
        owner._joint_state.velocity = [0.] * 7
        owner._joint_state_received = 100.
        owner._scene_robot = None
        owner._get_scene = None
        owner._apply_scene = None
        owner._state_validity = None
        self.links = ["ground/base_link", "ground/forearm_link", "ground/wrist_1_link",
                      "ground/left_outer_knuckle", "ground/ag95_body",
                      "ground/gripper_tcp_link"] + list(scene_helper.FINGER_LINKS)
        self.robot = types.SimpleNamespace(
            get_link_names=mock.Mock(return_value=self.links),
            get_active_joint_names=mock.Mock(return_value=owner._joint_state.name))
        self.scene = PlanningScene()
        floor = CollisionObject(id="floor")
        self.scene.world.collision_objects = [floor]
        self.events, self.get_requests, self.applied, self.validity_requests = [], [], [], []
        self.statuses = []
        self.action_goals = []
        self.valid = True
        self.apply_success = True
        self.actual_preshape = None
        self.close_failure = False
        self.lift_failure = False
        self.measured_tcp = target_pose(2.001, .098, .0819, .2)
        self.measured_tcp.pose.orientation.y = math.sqrt(.5)
        self.measured_tcp.pose.orientation.z = 0.
        self.measured_tcp.pose.orientation.w = math.sqrt(.5)

        def get_scene(request):
            self.get_requests.append(copy.deepcopy(request))
            return GetPlanningSceneResponse(scene=copy.deepcopy(self.scene))

        def apply_scene(request):
            change = request.scene
            self.applied.append(copy.deepcopy(change))
            if not self.apply_success:
                return ApplyPlanningSceneResponse(success=False)
            if change.allowed_collision_matrix.entry_names:
                self.scene.allowed_collision_matrix = copy.deepcopy(change.allowed_collision_matrix)
            for obj in change.world.collision_objects:
                self.scene.world.collision_objects = [
                    old for old in self.scene.world.collision_objects if old.id != obj.id]
                if obj.operation != CollisionObject.REMOVE:
                    self.scene.world.collision_objects.append(copy.deepcopy(obj))
                    self.events.append("target_update")
            for item in change.robot_state.attached_collision_objects:
                self.scene.world.collision_objects = [old for old in self.scene.world.collision_objects
                                                      if old.id != item.object.id]
                self.scene.robot_state.attached_collision_objects.append(copy.deepcopy(item))
                self.events.append("payload_added")
            return ApplyPlanningSceneResponse(success=True)

        def validity(request):
            self.validity_requests.append(copy.deepcopy(request))
            return GetStateValidityResponse(valid=self.valid)

        def proxy(name, _type):
            return {"/get_planning_scene": get_scene,
                    "/apply_planning_scene": apply_scene,
                    "/check_state_validity": validity}[name]

        def lookup(frame, child, stamp, timeout):
            result = TransformStamped()
            result.header.frame_id = frame
            result.child_frame_id = child
            result.header.stamp = rospy.Time.from_sec(10.)
            result.transform.rotation.w = 1.
            if child == owner._end_effector_link:
                p = self.measured_tcp.pose.position
                result.transform.translation.x = p.x
                result.transform.translation.y = p.y
                result.transform.translation.z = p.z
                result.transform.rotation = copy.deepcopy(self.measured_tcp.pose.orientation)
            return result

        owner._tf_buffer = types.SimpleNamespace(lookup_transform=mock.Mock(side_effect=lookup))
        owner._move_group = types.SimpleNamespace(get_planning_frame=lambda: "map")
        def publish(stage, **values):
            self.events.append(stage)
            self.statuses.append((stage, values))
        owner._publish_status = publish
        owner._feasibility = demo.check_target_feasibility(
            owner._target_size, owner._maximum_gripper_opening, owner._opening_margin)

        def wait_action(_timeout):
            q = self.action_goals[-1].trajectory.points[-1].positions[0]
            if q == owner._gripper_closed_position:
                self.events.append("close")
                if self.close_failure:
                    raise demo.DemoError("physical close failed")
                owner._joint_state.position[-1] = .45737440709566757
                owner._grasp_confirmed = True
                owner._grasp_confirmation_received = 100.
            else:
                self.events.append("preshape")
                owner._joint_state.position[-1] = q if self.actual_preshape is None else self.actual_preshape
            owner._joint_state_received = 100.
            return True

        owner._gripper_client = types.SimpleNamespace(
            wait_for_server=lambda timeout: True,
            send_goal=lambda goal: self.action_goals.append(copy.deepcopy(goal)),
            wait_for_result=wait_action,
            get_state=lambda: demo.GoalStatus.SUCCEEDED)
        self.patches = [mock.patch.object(demo.rospy.Time, "now", return_value=rospy.Time.from_sec(10.)),
                        mock.patch.object(demo.time, "monotonic", return_value=100.),
                        mock.patch.object(demo.rospy, "is_shutdown", return_value=False),
                        mock.patch.object(demo.rospy, "wait_for_service"),
                        mock.patch.object(demo.rospy, "ServiceProxy", side_effect=proxy),
                        mock.patch.object(demo.moveit_commander, "RobotCommander", return_value=self.robot)]
        for patch in self.patches:
            patch.start()
            self.addCleanup(patch.stop)

    def require_hook(self, name):
        self.assertTrue(hasattr(self.owner, name), name + " is not implemented")
        return getattr(self.owner, name)

    def update_target(self):
        return self.require_hook("_update_manipulation_target")(target_pose())

    def allowed_links(self):
        acm = self.scene.allowed_collision_matrix
        index = acm.entry_names.index(scene_helper.TARGET_ID)
        return {name for name, allowed in zip(acm.entry_names, acm.entry_values[index].enabled) if allowed}

    def test_configuration_is_opt_in_and_boolean(self):
        config = yaml.safe_load((PACKAGE / "config/demo.yaml").read_text())
        self.assertIs(False, config.get("full_robot_manipulation"))
        source = (PACKAGE / "scripts/run_air_ground_pick_demo.py").read_text()
        self.assertIn('rospy.get_param("~full_robot_manipulation", False)', source)

    def test_scene_snapshot_uses_explicit_components_and_full_robot_links(self):
        self.update_target()
        required = (PlanningSceneComponents.SCENE_SETTINGS | PlanningSceneComponents.ROBOT_STATE |
                    PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS |
                    PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
                    PlanningSceneComponents.ALLOWED_COLLISION_MATRIX | PlanningSceneComponents.TRANSFORMS)
        self.assertTrue(self.get_requests)
        self.assertEqual(required, self.get_requests[-1].components.components & required)
        self.robot.get_link_names.assert_called_once_with()
        self.assertEqual(set(), self.allowed_links())
        self.assertEqual(["floor", scene_helper.TARGET_ID], [o.id for o in self.scene.world.collision_objects])

    def test_failed_scene_apply_stops_before_robot_motion(self):
        self.apply_success = False
        self.require_hook("_update_manipulation_target")
        with self.assertRaisesRegex(demo.DemoError, "scene"):
            self.owner._update_manipulation_target(target_pose())
        self.assertEqual([], self.action_goals)

    def test_observation_installs_only_accepted_aerial_pose_in_both_modes(self):
        self.require_hook("_update_manipulation_target")
        target = (2., .1, .0575, .2)
        for mode in ("rm4d", "standoff"):
            self.owner._placement_mode = mode
            def observe(received):
                self.assertEqual(target, received)
                box = next(o for o in self.scene.world.collision_objects if o.id == scene_helper.TARGET_ID)
                self.assertEqual(2., box.primitive_poses[0].position.x)
                self.assertEqual(set(), self.allowed_links())
                return target_pose(), target
            self.owner._observe_ground_target_rm4d = observe
            self.owner._observe_ground_target_standoff = observe
            self.owner._ground_surface_cue = object()  # never consumed as cuboid geometry
            self.owner._observe_ground_target(target)

    def test_disabled_observation_never_accesses_scene_services(self):
        self.owner._full_robot_manipulation = False
        expected = (target_pose(), (2., .1, .0575, .2))
        self.owner._observe_ground_target_rm4d = lambda target: expected
        self.assertEqual(expected, self.owner._observe_ground_target(expected[1]))
        self.assertEqual([], self.get_requests)
        self.assertEqual([], self.applied)

    def test_enabled_legacy_joint_observation_uses_collision_aware_plan(self):
        self.update_target()
        trace = []
        group = self.owner._move_group
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.points = [JointTrajectoryPoint(positions=[0.] * 6)]
        group.set_start_state = lambda state: trace.append("start")
        group.set_joint_value_target = lambda target: trace.append("joint_target")
        group.plan = lambda: (trace.append("plan") or (True, trajectory))
        group.execute = lambda trajectory, wait: (trace.append("execute") or True)
        group.stop = lambda: None
        self.owner._execute_trajectory = lambda *args, **kwargs: trace.append("raw_arm_command")
        self.owner._arm_client = object()
        self.owner._wait_joint_target = lambda *args: dict(zip(
            self.owner._observation_joint_names, self.owner._observation_joint_positions))
        self.owner._move_to_ground_observation()
        self.assertEqual(["start", "joint_target", "plan", "execute"], trace)

    def test_target_transform_requires_fresh_current_planning_frame(self):
        self.require_hook("_update_manipulation_target")
        self.owner._move_group.get_planning_frame = lambda: "ground/base_link"
        original = self.owner._tf_buffer.lookup_transform.side_effect
        def shifted(*args):
            result = original(*args)
            result.transform.translation.x = -.4
            return result
        self.owner._tf_buffer.lookup_transform.side_effect = shifted
        self.update_target()
        self.owner._tf_buffer.lookup_transform.assert_called_with(
            "ground/base_link", "map", rospy.Time(0), rospy.Duration(.5))
        box = self.scene.world.collision_objects[-1]
        self.assertEqual("ground/base_link", box.header.frame_id)
        self.assertAlmostEqual(1.6, box.primitive_poses[0].position.x)
        def stale(*args):
            result = shifted(*args)
            result.header.stamp = rospy.Time.from_sec(1.)
            return result
        self.owner._tf_buffer.lookup_transform.side_effect = stale
        with self.assertRaisesRegex(demo.DemoError, "stale"):
            self.update_target()

    def test_preshape_uses_measured_start_and_original_tracking_tolerance(self):
        self.update_target()
        opening = self.require_hook("_preshape_gripper")()
        goal, = self.action_goals
        self.assertEqual(0., goal.trajectory.points[0].positions[0])
        self.assertEqual(0., goal.trajectory.points[0].time_from_start.to_sec())
        self.assertAlmostEqual(.29271008403361354, goal.trajectory.points[-1].positions[0])
        self.assertEqual(2., goal.trajectory.points[-1].time_from_start.to_sec())
        self.assertAlmostEqual(.06523655913978493, opening)
        self.assertEqual(.10, self.owner._gripper_joint_tolerance)
        self.assertEqual(set(), self.allowed_links())

    def test_structured_stages_describe_only_completed_checks(self):
        self.update_target()
        self.require_hook("_preshape_gripper")()
        self.owner._close_gripper()
        stages = dict(self.statuses)
        self.assertIn("GROUND_MANIPULATION_SCENE", stages)
        self.assertIn("GROUND_GRIPPER_PRESHAPE", stages)
        self.assertIn("GROUND_GRASP_GEOMETRY", stages)
        self.assertIn("GROUND_PAYLOAD_MODELED", stages)
        self.assertEqual("accepted", stages["GROUND_MANIPULATION_SCENE"]["source"])
        self.assertAlmostEqual(.06523655913978493, stages["GROUND_GRIPPER_PRESHAPE"]["measured_opening"])
        self.assertAlmostEqual(.45737440709566757, stages["GROUND_GRASP_GEOMETRY"]["contact_joint"])
        self.assertGreater(stages["GROUND_GRASP_GEOMETRY"]["checked_states"], 1)
        self.assertEqual(10., stages["GROUND_PAYLOAD_MODELED"]["measurement_stamp"])

    def test_preshape_equality_aperture_is_still_rejected(self):
        self.update_target()
        self.actual_preshape = .93 * (1. - .055/.0952)
        self.require_hook("_preshape_gripper")
        with self.assertRaisesRegex(demo.DemoError, "opening is too small"):
            self.owner._preshape_gripper()

    def test_invalid_full_robot_sweep_prevents_gripper_actuation(self):
        self.update_target()
        self.valid = False
        self.require_hook("_preshape_gripper")
        with self.assertRaisesRegex(demo.DemoError, "collision|invalid"):
            self.owner._preshape_gripper()
        self.assertEqual([], self.action_goals)
        self.assertEqual("", self.validity_requests[-1].group_name)

    def test_feedback_state_preserves_payload_and_is_diff(self):
        self.update_target()
        attachment = AttachedCollisionObject()
        attachment.object.id = "held_payload"
        self.scene.robot_state.attached_collision_objects.append(attachment)
        result = self.owner._robot_state_from_joint_feedback()
        self.assertTrue(result.is_diff)
        self.assertEqual(["held_payload"], [o.object.id for o in result.attached_collision_objects])
        self.assertEqual(self.owner._joint_state.position, result.joint_state.position)

    def test_real_close_has_narrow_contacts_then_measured_attachment(self):
        self.update_target()
        self.owner._joint_state.position[-1] = .29271008403361354
        self.owner._close_gripper()
        self.assertEqual(set(scene_helper.FINGER_LINKS), self.allowed_links())
        self.assertEqual([scene_helper.TARGET_ID],
                         [o.object.id for o in self.scene.robot_state.attached_collision_objects])
        self.assertEqual(["floor"], [o.id for o in self.scene.world.collision_objects])
        checks_before_payload = [request for request in self.validity_requests
                                 if not request.robot_state.attached_collision_objects]
        qs = [dict(zip(r.robot_state.joint_state.name, r.robot_state.joint_state.position))[
            "left_outer_knuckle_joint"] for r in checks_before_payload]
        self.assertGreater(len(qs), 1)
        self.assertAlmostEqual(.45737440709566757, max(qs))
        self.assertLess(max(qs), .70)
        self.assertTrue(self.validity_requests[-1].robot_state.attached_collision_objects)
        self.assertEqual("", self.validity_requests[-1].group_name)

    def test_failed_physical_close_restores_world_contact_checks(self):
        self.update_target()
        self.owner._joint_state.position[-1] = .29271008403361354
        self.close_failure = True
        with self.assertRaisesRegex(demo.DemoError, "physical close failed"):
            self.owner._close_gripper()
        self.assertEqual(set(), self.allowed_links())
        self.assertEqual([], self.scene.robot_state.attached_collision_objects)

    def test_cleanup_failure_reports_original_close_failure(self):
        self.update_target()
        original_apply = self.owner._apply_scene
        self.owner._apply_scene = lambda request: (
            ApplyPlanningSceneResponse(success=False) if "close" in self.events else original_apply(request))
        self.close_failure = True
        with self.assertRaisesRegex(demo.DemoError, "physical close failed.*cleanup failed"):
            self.owner._close_gripper()

    def test_invalid_nonfinger_closure_geometry_stops_before_close(self):
        self.update_target()
        self.valid = False
        with self.assertRaises(demo.DemoError):
            self.owner._close_gripper()
        self.assertEqual([], self.action_goals)
        self.assertEqual(set(), self.allowed_links())

    def test_stale_tcp_after_real_close_never_adds_payload(self):
        self.update_target()
        original = self.owner._tf_buffer.lookup_transform.side_effect
        def stale(*args):
            result = original(*args)
            result.header.stamp = rospy.Time.from_sec(1.)
            return result
        self.owner._tf_buffer.lookup_transform.side_effect = stale
        with self.assertRaisesRegex(demo.DemoError, "stale"):
            self.owner._close_gripper()
        self.assertEqual([], self.scene.robot_state.attached_collision_objects)
        self.assertEqual(set(), self.allowed_links())

    def test_pick_order_keeps_all_checks_until_after_descent_and_retains_payload_on_lift_failure(self):
        self.require_hook("_update_manipulation_target")
        def pregrasp(target, continuation=None):
            self.events.append("pregrasp_motion")
            self.assertEqual(set(), self.allowed_links())
            return target
        def cartesian(target, label):
            self.events.append(label)
            if label == "grasp approach":
                self.assertEqual(set(), self.allowed_links())
            else:
                self.assertTrue(self.scene.robot_state.attached_collision_objects)
                raise demo.DemoError("lift execution failed")
            return target
        self.owner._execute_pregrasp = pregrasp
        self.owner._execute_cartesian = cartesian
        with self.assertRaisesRegex(demo.DemoError, "lift execution failed"):
            self.owner._pick_and_lift(target_pose(), (2., .1, .0575, .2))
        expected = ["target_update", "preshape", "PREGRASP", "pregrasp_motion", "GRASP",
                    "grasp approach", "close", "payload_added", "LIFTING", "lift"]
        observable = [event for event in self.events if event not in (
            "GROUND_MANIPULATION_SCENE", "GROUND_GRIPPER_PRESHAPE", "GROUND_GRASP_GEOMETRY", "GROUND_PAYLOAD_MODELED")]
        self.assertEqual(expected, observable)
        self.assertTrue(self.scene.robot_state.attached_collision_objects)

    def test_cartesian_request_sets_payload_state_before_collision_aware_planning(self):
        self.update_target()
        attachment = AttachedCollisionObject()
        attachment.object.id = scene_helper.TARGET_ID
        self.scene.robot_state.attached_collision_objects = [attachment]
        trace = []
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = list(self.owner._observation_joint_names)
        trajectory.joint_trajectory.points = [JointTrajectoryPoint(positions=[0.] * 6)]
        group = self.owner._move_group
        group.set_start_state = lambda state: trace.append(("start", copy.deepcopy(state)))
        group.compute_cartesian_path = lambda *args: (trace.append(("compute", args)) or (trajectory, 1.))
        group.retime_trajectory = lambda state, traj, *scales: traj
        group.execute = lambda traj, wait: True
        group.stop = lambda: None
        self.owner._verify_tcp_pose = lambda target, label: target
        self.owner._execute_cartesian(target_pose(), "lift")
        self.assertEqual("start", trace[0][0])
        self.assertTrue(trace[0][1].is_diff)
        self.assertEqual(scene_helper.TARGET_ID, trace[0][1].attached_collision_objects[0].object.id)
        self.assertTrue(trace[1][1][2])


if __name__ == "__main__":
    unittest.main()
