#pragma once

#include <brick_pick_demo/brick_geometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>

namespace brick_pick_demo {

struct GraspParameters {
  double pre_grasp_height{0.15};
  double lift_height{0.15};
  // At the real open configuration, the unchanged AG95 pad collision mesh
  // begins this far behind the nominal fingertip TCP along the approach axis.
  double finger_pad_lower_edge_offset{0.0156};
  // Required pad engagement below the upper side edge of the Brick.
  double finger_pad_contact_overlap{0.020};
  // Minimum TCP clearance above the Brick support plane during insertion.
  double surface_clearance{0.010};
  bool grasp_along_short_axis{true};
  std::string planning_frame{"world"};
};

struct GraspPoses {
  geometry_msgs::PoseStamped pre_grasp;
  geometry_msgs::PoseStamped grasp;
  geometry_msgs::PoseStamped lift;
};

bool generateTopDownGrasp(const geometry_msgs::PoseStamped& brick,
                          const BrickDimensions& dimensions,
                          const GraspParameters& parameters,
                          GraspPoses* result,
                          std::string* error);

}  // namespace brick_pick_demo
