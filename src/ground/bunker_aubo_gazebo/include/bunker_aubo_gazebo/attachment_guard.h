#pragma once

#include <string>

namespace bunker_aubo_gazebo {

struct AttachmentEvidence {
  bool geometry_feasible{false};
  bool preopened{false};
  bool gripper_closed{false};
  bool left_contact{false};
  bool right_contact{false};
  unsigned int left_contact_count{0};
  unsigned int right_contact_count{0};
  bool brick_bracketed{false};
  bool opposed_contact_normals{false};
  double pad_center_separation{0.0};
  double brick_axis_fraction{0.0};
  double maximum_penetration{0.0};
};

struct AttachmentDecision {
  bool allowed{false};
  std::string reason;
};

AttachmentDecision decideAttachment(const AttachmentEvidence& evidence,
                                    double penetration_limit);

}  // namespace bunker_aubo_gazebo
