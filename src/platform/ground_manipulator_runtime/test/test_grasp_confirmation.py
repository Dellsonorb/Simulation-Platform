#!/usr/bin/env python3

import importlib.util
from pathlib import Path
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
MODULE = (
    PACKAGE / "src/ground_manipulator_runtime/grasp_confirmation.py")
SCRIPT = PACKAGE / "scripts/gazebo_grasp_confirmation.py"


def load_module():
    spec = importlib.util.spec_from_file_location(
        "ground_grasp_confirmation_test_target", str(MODULE))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class GraspConfirmationTest(unittest.TestCase):
    def test_strict_bilateral_contact_and_closed_joint_are_required(self):
        module = load_module()
        evaluator = module.GraspConfirmationEvaluator(
            contact_max_age=0.30, joint_max_age=0.50,
            minimum_closed_joint=0.20)
        target = "pick_target::pick_target_link::pick_target_collision"
        left = "ground_robot::ground/left_finger_pad::collision"
        right = "ground_robot::ground/right_finger_pad::collision"

        evaluator.update_joint_state(
            ("left_outer_knuckle_joint",), (0.25,), received_at=1.0)
        evaluator.update_contacts(((target, left),), received_at=1.0)
        self.assertFalse(evaluator.confirmed(now=1.0))
        evaluator.update_contacts(((right, target),), received_at=1.1)
        self.assertTrue(evaluator.confirmed(now=1.1))
        self.assertFalse(evaluator.confirmed(now=1.41))

    def test_open_missing_or_invalid_joint_state_never_confirms(self):
        module = load_module()
        target = "pick_target::pick_target_link::pick_target_collision"
        left = "ground_robot::ground/left_finger_pad::collision"
        right = "ground_robot::ground/right_finger_pad::collision"
        evaluator = module.GraspConfirmationEvaluator(0.30, 0.50, 0.20)
        evaluator.update_contacts(
            ((target, left), (target, right)), received_at=2.0)
        self.assertFalse(evaluator.confirmed(now=2.0))
        evaluator.update_joint_state(
            ("left_outer_knuckle_joint",), (0.19,), received_at=2.0)
        self.assertFalse(evaluator.confirmed(now=2.0))
        with self.assertRaises(module.GraspConfirmationError):
            evaluator.update_joint_state(
                ("left_outer_knuckle_joint",), (float("nan"),),
                received_at=2.0)

    def test_only_sim_adapter_knows_gazebo_contact_details(self):
        source = SCRIPT.read_text(encoding="utf-8")
        self.assertIn("from gazebo_msgs.msg import ContactsState", source)
        self.assertIn('"/pick_target/contacts"', source)
        self.assertIn('"/ground/joint_states"', source)
        self.assertIn('"/ground/gripper/grasp_confirmed"', source)
        self.assertIn("std_msgs.msg import Bool", source)
        self.assertNotIn("attach", source.lower())
        self.assertNotIn("set_model_state", source.lower())


if __name__ == "__main__":
    unittest.main()
