#pragma once

#include <brick_pick_demo/grasp_geometry.h>

#include <string>

namespace brick_pick_demo {

struct GripperFeasibility {
  bool feasible{false};
  double grasp_span{0.0};
  double required_opening{0.0};
  std::string reason;
};

GripperFeasibility checkGripperOpening(const BrickDimensions& dimensions,
                                       bool grasp_along_short_axis,
                                       double maximum_opening,
                                       double clearance_margin);

bool estimateJawOpening(double master_joint_position, double maximum_opening,
                        double maximum_joint_position, double* opening,
                        std::string* error);

}  // namespace brick_pick_demo
