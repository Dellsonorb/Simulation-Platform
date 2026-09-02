"""Pure strict grasp evaluator used only by the Gazebo SIM adapter."""

import math


class GraspConfirmationError(ValueError):
    pass


def _finite(value, label):
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise GraspConfirmationError("%s must be finite" % label) from error
    if not math.isfinite(result):
        raise GraspConfirmationError("%s must be finite" % label)
    return result


def contact_sides(
        contact_pairs, target_marker="pick_target",
        left_pad_marker="left_finger_pad",
        right_pad_marker="right_finger_pad"):
    """Classify target contact pairs inside the Gazebo backend boundary."""
    left = False
    right = False
    for pair in contact_pairs:
        try:
            first, second = pair
        except (TypeError, ValueError) as error:
            raise GraspConfirmationError(
                "contact pair must contain two names") from error
        first = str(first)
        second = str(second)
        if target_marker in first:
            other = second
        elif target_marker in second:
            other = first
        else:
            continue
        left = left or left_pad_marker in other
        right = right or right_pad_marker in other
    return left, right


class GraspConfirmationEvaluator:
    def __init__(
            self, contact_max_age, joint_max_age, minimum_closed_joint,
            joint_name="left_outer_knuckle_joint"):
        self.contact_max_age = _finite(contact_max_age, "contact_max_age")
        self.joint_max_age = _finite(joint_max_age, "joint_max_age")
        self.minimum_closed_joint = _finite(
            minimum_closed_joint, "minimum_closed_joint")
        if (self.contact_max_age <= 0.0 or self.joint_max_age <= 0.0 or
                self.minimum_closed_joint < 0.0 or not joint_name):
            raise GraspConfirmationError("grasp thresholds are invalid")
        self.joint_name = str(joint_name)
        self.left_received = None
        self.right_received = None
        self.joint_received = None
        self.joint_position = None

    def update_contacts(self, contact_pairs, received_at):
        received_at = _finite(received_at, "contact receipt time")
        left, right = contact_sides(contact_pairs)
        if left:
            self.left_received = received_at
        if right:
            self.right_received = received_at

    def update_joint_state(self, names, positions, received_at):
        received_at = _finite(received_at, "joint receipt time")
        names = tuple(str(name) for name in names)
        positions = tuple(positions)
        if len(names) != len(positions) or self.joint_name not in names:
            return
        index = names.index(self.joint_name)
        self.joint_position = _finite(
            positions[index], "%s position" % self.joint_name)
        self.joint_received = received_at

    @staticmethod
    def _fresh(received, now, maximum_age):
        return bool(
            received is not None and now >= received and
            now - received <= maximum_age)

    def confirmed(self, now):
        now = _finite(now, "current time")
        return bool(
            self._fresh(self.left_received, now, self.contact_max_age) and
            self._fresh(self.right_received, now, self.contact_max_age) and
            self._fresh(self.joint_received, now, self.joint_max_age) and
            self.joint_position is not None and
            self.joint_position >= self.minimum_closed_joint)
