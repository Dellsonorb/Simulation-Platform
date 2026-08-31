#include <bunker_aubo_gazebo/attachment_guard.h>

#include <cmath>

namespace bunker_aubo_gazebo {

AttachmentDecision decideAttachment(const AttachmentEvidence& evidence,
                                    double penetration_limit) {
  if (!evidence.geometry_feasible) {
    return {false, "PHYSICALLY_INFEASIBLE_WITH_CURRENT_GRASP"};
  }
  if (!evidence.preopened) {
    return {false, "gripper was not correctly pre-opened"};
  }
  if (!evidence.gripper_closed) {
    return {false, "gripper is not closed"};
  }
  if (!evidence.left_contact || !evidence.right_contact ||
      evidence.left_contact_count == 0 ||
      evidence.right_contact_count == 0) {
    return {false, "bilateral finger-pad contact is missing"};
  }
  if (!evidence.brick_bracketed) {
    return {false, "brick center is not bracketed by the finger pads"};
  }
  if (!evidence.opposed_contact_normals) {
    return {false, "finger-pad contact normals are not opposed"};
  }
  if (!std::isfinite(evidence.maximum_penetration) ||
      !std::isfinite(penetration_limit) || penetration_limit < 0.0 ||
      evidence.maximum_penetration > penetration_limit) {
    return {false, "finger/brick penetration exceeds guard"};
  }
  return {true, "contact-qualified brick attachment"};
}

}  // namespace bunker_aubo_gazebo
