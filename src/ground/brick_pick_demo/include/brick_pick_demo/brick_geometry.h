#pragma once

#include <string>

namespace brick_pick_demo {

struct BrickDimensions {
  double length{0.240};
  double width{0.115};
  double height{0.053};
  std::string orientation_mode{"flat"};
};

struct OrientedBrickGeometry {
  double top_length{0.0};
  double top_width{0.0};
  double vertical_height{0.0};
  double grasp_span{0.0};
  double resting_center_z{0.0};
};

bool deriveOrientedBrickGeometry(const BrickDimensions& dimensions,
                                 OrientedBrickGeometry* result,
                                 std::string* error);

}  // namespace brick_pick_demo
