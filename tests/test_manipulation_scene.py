"""Native-message, offline tests: these do not start ROS or apply a scene."""

import copy
import importlib.util
import io
import math
from pathlib import Path
import sys
import shlex
import struct
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/demos/air_ground_pick_demo/src"))
try:
    from geometry_msgs.msg import PoseStamped
    from moveit_msgs.msg import (AllowedCollisionEntry, AttachedCollisionObject,
                                 CollisionObject, PlanningScene, RobotState)
    from sensor_msgs.msg import JointState
    from shape_msgs.msg import SolidPrimitive
    from air_ground_pick_demo.grasp import quaternion_matrix
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


def pose(x=2.0, y=0.1, z=0.0575, yaw=0.2, frame="map"):
    result = PoseStamped()
    result.header.frame_id = frame
    result.header.stamp.secs = 12
    result.pose.position.x, result.pose.position.y, result.pose.position.z = x, y, z
    result.pose.orientation.z = math.sin(yaw / 2.0)
    result.pose.orientation.w = math.cos(yaw / 2.0)
    return result


def matrix(value):
    result = np.eye(4)
    q = value.orientation
    result[:3, :3] = quaternion_matrix((q.x, q.y, q.z, q.w))
    result[:3, 3] = [value.position.x, value.position.y, value.position.z]
    return result


NATIVE_SCENE = r'''
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/attached_body.h>
#include <urdf_parser/urdf_parser.h>
#include <ros/serialization.h>
#include <iostream>
#include <iomanip>
#include <vector>
moveit_msgs::PlanningScene readMessage() {
  uint32_t n; std::cin.read(reinterpret_cast<char*>(&n), 4);
  std::vector<uint8_t> buffer(n); std::cin.read(reinterpret_cast<char*>(buffer.data()), n);
  ros::serialization::IStream stream(buffer.data(), n);
  moveit_msgs::PlanningScene result; ros::serialization::deserialize(stream, result); return result;
}
void printPose(const geometry_msgs::Pose& p) {
  std::cout << p.position.x << " " << p.position.y << " " << p.position.z << " "
            << p.orientation.x << " " << p.orientation.y << " " << p.orientation.z << " " << p.orientation.w << " ";
}
int main(int argc, char**) {
  auto robot = urdf::parseURDF("<robot name='probe'><link name='ground/base_link'/><link name='ground/gripper_tcp_link'/><joint name='tcp' type='fixed'><parent link='ground/base_link'/><child link='ground/gripper_tcp_link'/><origin xyz='1.98 .12 .09' rpy='0 0 -.3'/></joint></robot>");
  auto semantic = std::make_shared<srdf::Model>(); semantic->initString(*robot, "<robot name='probe'/>");
  planning_scene::PlanningScene scene(robot, semantic);
  bool added = scene.setPlanningSceneDiffMsg(readMessage());
  moveit_msgs::CollisionObject target; scene.getCollisionObjectMsg(target, "perceived_pick_target");
  std::cout << std::setprecision(17) << "ROUNDTRIP " << added << " ";
  printPose(target.pose); printPose(target.primitive_poses.at(0)); std::cout << "\n";
  if (argc > 1) {
    bool applied = scene.setPlanningSceneDiffMsg(readMessage());
    const auto* body = scene.getCurrentState().getAttachedBody("perceived_pick_target");
    std::cout << "APPLIED " << applied << " ATTACHED " << bool(body) << " WORLD "
              << scene.getWorld()->hasObject("perceived_pick_target") << "\n";
    if (body) {
      std::cout << "GEOMETRY ";
      const auto& tf = body->getGlobalCollisionBodyTransforms().at(0).matrix();
      for (int i=0; i<4; ++i) for (int j=0; j<4; ++j) std::cout << tf(i,j) << " ";
      std::cout << "\n";
    }
  }
}
'''


@unittest.skipUnless(ROS_AVAILABLE, "native ROS messages required")
class ManipulationSceneTest(unittest.TestCase):
    LINKS = ("ground/base_link", "ground/forearm_link", "ground/wrist_1_link", "ground/left_outer_knuckle",
             "ground/left_finger", "ground/right_finger", "ground/left_finger_pad",
             "ground/right_finger_pad", "ground/gripper_tcp_link")

    def setUp(self):
        spec = importlib.util.find_spec("air_ground_pick_demo.manipulation_scene")
        self.assertIsNotNone(spec, "perceived manipulation scene helper is not implemented")
        from air_ground_pick_demo import manipulation_scene
        self.helper = manipulation_scene
        self.scene = PlanningScene()
        self.scene.name = "existing_full_robot_scene"
        floor = CollisionObject()
        floor.id = "floor"
        self.scene.world.collision_objects = [floor]
        carried = AttachedCollisionObject()
        carried.object.id = "unrelated_attachment"
        carried.link_name = "ground/wrist_1_link"
        self.scene.robot_state.attached_collision_objects = [carried]
        acm = self.scene.allowed_collision_matrix
        acm.entry_names = ["ground/base_link", "ground/forearm_link", "floor"]
        acm.entry_values = [AllowedCollisionEntry([False, True, False]),
                            AllowedCollisionEntry([True, False, False]),
                            AllowedCollisionEntry([False, False, False])]
        acm.default_entry_names = ["unrelated_default", self.helper.TARGET_ID]
        acm.default_entry_values = [True, True]

    def world(self):
        return self.helper.world_target_diff(
            self.scene, pose(), (.24, .053, .115), self.LINKS)

    @staticmethod
    def allowed(acm, first, second):
        return acm.entry_values[acm.entry_names.index(first)].enabled[
            acm.entry_names.index(second)]

    def with_target(self):
        result = copy.deepcopy(self.scene)
        change = self.world()
        result.world.collision_objects.extend(change.world.collision_objects)
        result.allowed_collision_matrix = change.allowed_collision_matrix
        return result

    def test_world_update_uses_only_accepted_pose_and_exact_dimensions(self):
        before = copy.deepcopy(self.scene)
        change = self.world()
        self.assertTrue(change.is_diff)
        self.assertTrue(change.robot_state.is_diff)
        self.assertEqual([], change.robot_state.attached_collision_objects)
        self.assertEqual(1, len(change.world.collision_objects))
        target = change.world.collision_objects[0]
        self.assertEqual(self.helper.TARGET_ID, target.id)
        self.assertEqual(CollisionObject.ADD, target.operation)
        self.assertEqual(1.0, target.pose.orientation.w)
        self.assertEqual(pose().header, target.header)
        self.assertEqual(pose().pose, target.primitive_poses[0])
        self.assertEqual(SolidPrimitive.BOX, target.primitives[0].type)
        self.assertEqual([.24, .053, .115], target.primitives[0].dimensions)
        self.assertEqual(before, self.scene)

    def test_world_update_explicitly_forbids_every_target_robot_pair(self):
        acm = self.world().allowed_collision_matrix
        for link in self.LINKS:
            self.assertFalse(self.allowed(acm, self.helper.TARGET_ID, link))
            self.assertFalse(self.allowed(acm, link, self.helper.TARGET_ID))
        defaults = dict(zip(acm.default_entry_names, acm.default_entry_values))
        self.assertFalse(defaults[self.helper.TARGET_ID])
        self.assertTrue(defaults["unrelated_default"])

    def test_acm_preserves_unrelated_explicit_pairs_and_default_semantics(self):
        acm = self.scene.allowed_collision_matrix
        acm.default_entry_names += ["ground/left_finger", "ground/right_finger"]
        acm.default_entry_values += [True, False]
        change = self.world().allowed_collision_matrix
        for i, first in enumerate(acm.entry_names):
            for j, second in enumerate(acm.entry_names):
                self.assertEqual(acm.entry_values[i].enabled[j],
                                 self.allowed(change, first, second))
        self.assertTrue(self.allowed(change, "ground/left_finger", "floor"))
        self.assertFalse(self.allowed(change, "ground/left_finger", "ground/right_finger"))

    def test_grasp_contact_allows_only_four_finger_assembly_links_then_restores_checks(self):
        current = self.with_target()
        change = self.helper.finger_contact_diff(current, self.LINKS, True)
        self.assertEqual([], change.world.collision_objects)
        self.assertEqual([], change.robot_state.attached_collision_objects)
        for link in self.LINKS:
            self.assertEqual(link in ("ground/left_finger", "ground/right_finger",
                                     "ground/left_finger_pad", "ground/right_finger_pad"),
                             self.allowed(change.allowed_collision_matrix,
                                          self.helper.TARGET_ID, link))
        self.assertFalse(self.allowed(change.allowed_collision_matrix,
                                      self.helper.TARGET_ID, "floor"))
        current.allowed_collision_matrix = change.allowed_collision_matrix
        restored = self.helper.finger_contact_diff(current, self.LINKS, False)
        for link in self.LINKS:
            self.assertFalse(self.allowed(restored.allowed_collision_matrix,
                                          self.helper.TARGET_ID, link))

    def test_refine_update_replaces_same_id_and_resets_contact_permissions(self):
        current = self.with_target()
        current.allowed_collision_matrix = self.helper.finger_contact_diff(
            current, self.LINKS, True).allowed_collision_matrix
        refined = pose(2.05, .08, .059, .056)
        change = self.helper.world_target_diff(
            current, refined, (.24, .053, .115), self.LINKS)
        self.assertEqual(self.helper.TARGET_ID, change.world.collision_objects[0].id)
        self.assertEqual(refined.pose, change.world.collision_objects[0].primitive_poses[0])
        self.assertFalse(self.allowed(change.allowed_collision_matrix,
                                      self.helper.TARGET_ID, "ground/left_finger"))

    def test_attach_uses_measured_tcp_inverse_and_preserves_world_and_state(self):
        current = self.with_target()
        before = copy.deepcopy(current)
        tcp = pose(1.98, .12, .09, -.3)
        # Non-yaw TCP orientation exercises the real top-down attachment frame.
        tcp.pose.orientation.x = .1
        tcp.pose.orientation.y = .7
        q = tcp.pose.orientation
        norm = math.sqrt(sum(v*v for v in (q.x, q.y, q.z, q.w)))
        q.x, q.y, q.z, q.w = [v/norm for v in (q.x, q.y, q.z, q.w)]
        change = self.helper.attach_target_diff(
            current, tcp, "ground/gripper_tcp_link", self.LINKS, grasp_confirmed=True)
        self.assertTrue(change.is_diff)
        self.assertTrue(change.robot_state.is_diff)
        self.assertEqual([], change.world.collision_objects)
        attachment, = change.robot_state.attached_collision_objects
        self.assertEqual("ground/gripper_tcp_link", attachment.link_name)
        self.assertEqual("ground/gripper_tcp_link", attachment.object.header.frame_id)
        self.assertEqual(["ground/left_finger", "ground/right_finger", "ground/left_finger_pad",
                          "ground/right_finger_pad"], attachment.touch_links)
        self.assertEqual([.24, .053, .115], attachment.object.primitives[0].dimensions)
        np.testing.assert_allclose(
            matrix(tcp.pose) @ matrix(attachment.object.pose) @ matrix(attachment.object.primitive_poses[0]),
            matrix(pose().pose), atol=1e-12)
        self.assertFalse(self.allowed(change.allowed_collision_matrix,
                                      self.helper.TARGET_ID, "ground/base_link"))
        self.assertEqual(before, current)

    def test_attach_composes_nonidentity_object_and_primitive_poses(self):
        current = self.with_target()
        target = current.world.collision_objects[-1]
        target.pose = pose(1.2, -.7, .2, .3).pose
        tcp = pose(1.98, .12, .09, -.3)
        expected = matrix(target.pose) @ matrix(target.primitive_poses[0])
        change = self.helper.attach_target_diff(
            current, tcp, "ground/gripper_tcp_link", self.LINKS, True)
        attachment = change.robot_state.attached_collision_objects[0].object
        actual = matrix(tcp.pose) @ matrix(attachment.pose) @ matrix(attachment.primitive_poses[0])
        np.testing.assert_allclose(expected, actual, atol=1e-12)

    def test_native_planning_scene_roundtrip_and_attachment_apply(self):
        flags = shlex.split(subprocess.check_output(["pkg-config", "--cflags", "moveit_core"], text=True))
        libraries = shlex.split(subprocess.check_output(["pkg-config", "--libs", "moveit_core"], text=True))
        def packet(message):
            stream = io.BytesIO()
            message.serialize(stream)
            data = stream.getvalue()
            return struct.pack("<I", len(data)) + data
        def assign_pose(message, values):
            message.position.x, message.position.y, message.position.z = values[:3]
            (message.orientation.x, message.orientation.y,
             message.orientation.z, message.orientation.w) = values[3:]
        with tempfile.TemporaryDirectory(prefix="p450-native-scene-") as directory:
            executable = str(Path(directory) / "scene-roundtrip")
            built = subprocess.run(["g++", "-std=c++14"] + flags +
                ["-x", "c++", "-", "-x", "none", "-o", executable] + libraries,
                input=NATIVE_SCENE, text=True, capture_output=True, timeout=60)
            self.assertEqual(0, built.returncode, built.stderr)
            for object_layout in (False, True):
                with self.subTest(object_layout=object_layout):
                    accepted = pose(frame="ground/base_link")
                    change = self.helper.world_target_diff(PlanningScene(), accepted,
                        (.24, .053, .115), self.LINKS)
                    if object_layout:
                        target = change.world.collision_objects[0]
                        target.pose = copy.deepcopy(accepted.pose)
                        target.primitive_poses[0] = pose(0., 0., 0., 0.).pose
                    first = subprocess.run([executable], input=packet(change), capture_output=True, timeout=10)
                    self.assertEqual(0, first.returncode, first.stderr)
                    line = next(line for line in first.stdout.decode().splitlines() if line.startswith("ROUNDTRIP "))
                    self.assertEqual("1", line.split()[1])
                    values = list(map(float, line.split()[2:]))
                    roundtrip = copy.deepcopy(change.world.collision_objects[0])
                    assign_pose(roundtrip.pose, values[:7])
                    assign_pose(roundtrip.primitive_poses[0], values[7:])
                    current = PlanningScene()
                    current.world.collision_objects = [roundtrip]
                    current.allowed_collision_matrix = change.allowed_collision_matrix
                    tcp = pose(1.98, .12, .09, -.3, "ground/base_link")
                    attachment = self.helper.attach_target_diff(current, tcp,
                        "ground/gripper_tcp_link", self.LINKS, True)
                    second = subprocess.run([executable, "attach"], input=packet(change)+packet(attachment),
                                            capture_output=True, timeout=10)
                    self.assertEqual(0, second.returncode, second.stderr)
                    lines = second.stdout.decode().splitlines()
                    self.assertIn("APPLIED 1 ATTACHED 1 WORLD 0", lines)
                    actual = next(line for line in lines if line.startswith("GEOMETRY "))
                    actual = np.array(list(map(float, actual.split()[1:]))).reshape(4, 4)
                    np.testing.assert_allclose(matrix(accepted.pose), actual, atol=1e-12)

    def test_attachment_requires_real_grasp_and_matching_measured_frame(self):
        current = self.with_target()
        for confirmed, tcp, link in ((False, pose(), "ground/gripper_tcp_link"),
                                      (True, pose(frame="odom"), "ground/gripper_tcp_link"),
                                      (True, pose(), "not_in_robot")):
            with self.subTest(confirmed=confirmed, frame=tcp.header.frame_id, link=link):
                with self.assertRaises(self.helper.SceneError):
                    self.helper.attach_target_diff(current, tcp, link, self.LINKS, confirmed)
        with self.assertRaises(self.helper.SceneError):
            self.helper.attach_target_diff(self.scene, pose(), "ground/gripper_tcp_link", self.LINKS, True)

    def test_world_update_does_not_overwrite_an_attached_target(self):
        attached = AttachedCollisionObject()
        attached.object.id = self.helper.TARGET_ID
        self.scene.robot_state.attached_collision_objects.append(attached)
        with self.assertRaises(self.helper.SceneError):
            self.world()

    def test_contact_scope_matches_actual_complete_rendered_robot_model(self):
        from ground_manipulator_runtime.renderer import render_ground_robot
        robot = ET.fromstring(render_ground_robot(
            ROOT / "src/platform/ground_manipulator_runtime/urdf/ground_robot.urdf.xacro"))
        links = [link.get("name") for link in robot.findall("link")]
        current = copy.deepcopy(self.scene)
        world = self.helper.world_target_diff(current, pose(), (.24, .053, .115), links)
        current.world.collision_objects.extend(world.world.collision_objects)
        current.allowed_collision_matrix = world.allowed_collision_matrix
        contact = self.helper.finger_contact_diff(current, links, True)
        expected = {"ground/left_finger", "ground/right_finger",
                    "ground/left_finger_pad", "ground/right_finger_pad"}
        actual = {link for link in links if self.allowed(
            contact.allowed_collision_matrix, self.helper.TARGET_ID, link)}
        self.assertEqual(expected, actual)
        for link in links:
            self.assertFalse(self.allowed(world.allowed_collision_matrix,
                                          self.helper.TARGET_ID, link))
        for first in links:
            for second in links:
                self.assertEqual(self.allowed(world.allowed_collision_matrix, first, second),
                                 self.allowed(contact.allowed_collision_matrix, first, second))

    def test_contact_toggle_requires_known_target(self):
        with self.assertRaises(self.helper.SceneError):
            self.helper.finger_contact_diff(self.scene, self.LINKS, True)

    def test_invalid_geometry_and_frames_fail_closed(self):
        invalid_pose = pose()
        invalid_pose.pose.orientation.w = 0.0
        invalid_pose.pose.orientation.z = 0.0
        for candidate, size in ((pose(frame=""), (.24, .053, .115)),
                                (pose(x=float("nan")), (.24, .053, .115)),
                                (invalid_pose, (.24, .053, .115)),
                                (pose(), (.24, -.053, .115)),
                                (pose(), (.24, float("nan"), .115))):
            with self.subTest(candidate=candidate, size=size):
                with self.assertRaises(self.helper.SceneError):
                    self.helper.world_target_diff(self.scene, candidate, size, self.LINKS)

    def test_invalid_acm_or_robot_link_list_is_rejected(self):
        for links in ((), ("ground/base_link",), self.LINKS + ("ground/base_link",)):
            with self.subTest(links=links):
                with self.assertRaises(self.helper.SceneError):
                    self.helper.world_target_diff(self.scene, pose(), (.24, .053, .115), links)
        self.scene.allowed_collision_matrix.entry_values[0].enabled.pop()
        with self.assertRaises(self.helper.SceneError):
            self.world()

    def test_start_state_keeps_payload_and_multidof_with_measured_joints(self):
        current = self.scene.robot_state
        current.multi_dof_joint_state.joint_names = ["world_joint"]
        measured = JointState()
        measured.name = ["wrist1_joint", "left_outer_knuckle_joint"]
        measured.position = [.3, .4]
        measured.velocity = [.0, .0]
        result = self.helper.planning_start_state(current, measured)
        self.assertTrue(result.is_diff)
        self.assertEqual(current.attached_collision_objects, result.attached_collision_objects)
        self.assertEqual(current.multi_dof_joint_state, result.multi_dof_joint_state)
        self.assertEqual(measured, result.joint_state)
        result.attached_collision_objects[0].object.id = "changed"
        result.joint_state.position[0] = .5
        self.assertEqual("unrelated_attachment", current.attached_collision_objects[0].object.id)
        self.assertEqual(.3, measured.position[0])

    def test_start_state_rejects_invalid_measured_joint_positions(self):
        for names, values in (([], []), (["j"], []), (["j", "j"], [0., 0.]),
                              (["j"], [float("nan")])):
            message = JointState(name=names, position=values)
            with self.subTest(names=names, values=values):
                with self.assertRaises(self.helper.SceneError):
                    self.helper.planning_start_state(self.scene.robot_state, message)

    def test_preshape_is_inverse_calibration_minus_existing_tracking_tolerance(self):
        result = self.helper.required_opening_preshape(
            (.24, .053, .115), .0952, .93, .002, .10)
        self.assertAlmostEqual(.055, result.required_opening)
        self.assertAlmostEqual(.39271008403361354, result.closure_limit)
        self.assertAlmostEqual(.29271008403361354, result.command_position)
        self.assertAlmostEqual(.06523655913978493, result.predicted_opening)
        self.assertGreater(result.predicted_opening, result.required_opening)
        # The exact +tolerance endpoint is equality, not an accepted aperture.
        boundary = .0952 * (1.0 - (result.command_position + .10)/.93)
        self.assertAlmostEqual(result.required_opening, boundary)

    def test_preshape_rejects_infeasible_target_or_missing_tracking_room(self):
        for size, opening, limit, margin, tracking in (
                ((.24, .10, .115), .0952, .93, .002, .10),
                ((.24, .09, .115), .0952, .93, .002, .10),
                ((.24, .053, .115), .0952, .93, .002, float("nan")),
                ((.24, .053, .115), .0952, .93, .002, -.1)):
            with self.subTest(size=size, tracking=tracking):
                with self.assertRaises(self.helper.SceneError):
                    self.helper.required_opening_preshape(size, opening, limit, margin, tracking)


if __name__ == "__main__":
    unittest.main()
