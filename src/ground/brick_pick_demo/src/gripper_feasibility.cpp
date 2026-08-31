#include <brick_pick_demo/gripper_feasibility.h>

#include <algorithm>
#include <cmath>

namespace brick_pick_demo {

GripperFeasibility checkGripperOpening(const BrickDimensions& dimensions,
                                       bool grasp_along_short_axis,
                                       double maximum_opening,
                                       double clearance_margin) {
  GripperFeasibility result;
  OrientedBrickGeometry oriented;
  std::string geometry_error;
  if (!deriveOrientedBrickGeometry(dimensions, &oriented, &geometry_error)) {
    result.reason = geometry_error;
    return result;
  }
  result.grasp_span = grasp_along_short_axis
                          ? oriented.grasp_span
                          : oriented.top_length;
  result.required_opening = result.grasp_span + clearance_margin;
  if (!std::isfinite(result.required_opening) ||
      !std::isfinite(maximum_opening) ||
      !std::isfinite(clearance_margin) || result.grasp_span <= 0.0 ||
      maximum_opening <= 0.0 || clearance_margin < 0.0) {
    result.reason = "INVALID_GRIPPER_GEOMETRY";
    return result;
  }
  result.feasible =
      result.required_opening <= maximum_opening;
  if (!result.feasible) {
    result.reason = "PHYSICALLY_INFEASIBLE_WITH_CURRENT_GRASP";
  }
  return result;
}

bool estimateJawOpening(double master_joint_position, double maximum_opening,
                        double maximum_joint_position, double* opening,
                        std::string* error) {
  if (!opening) {
    if (error) *error = "INVALID_GRIPPER_OPENING_OUTPUT";
    return false;
  }
  if (!std::isfinite(maximum_opening) ||
      !std::isfinite(maximum_joint_position) || maximum_opening <= 0.0 ||
      maximum_joint_position <= 0.0) {
    if (error) *error = "INVALID_GRIPPER_CALIBRATION";
    return false;
  }
  if (!std::isfinite(master_joint_position) || master_joint_position < -1e-4 ||
      master_joint_position > maximum_joint_position + 1e-4) {
    if (error) *error = "INVALID_GRIPPER_JOINT_MEASUREMENT";
    return false;
  }
  const double bounded_joint = std::max(
      0.0, std::min(maximum_joint_position, master_joint_position));
  // The chord between the measured collision gap at both real URDF limits is
  // below the sampled AG95 mesh gap through the stroke, so this is a
  // conservative lower bound rather than an enlarged simulated aperture.
  *opening = maximum_opening *
             (1.0 - bounded_joint / maximum_joint_position);
  if (error) error->clear();
  return true;
}

}  // namespace brick_pick_demo
