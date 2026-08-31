#include <brick_pick_demo/grasp_geometry.h>

#include <cmath>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace brick_pick_demo {

bool generateTopDownGrasp(const geometry_msgs::PoseStamped& brick,
                          const BrickDimensions& dimensions,
                          const GraspParameters& parameters,
                          GraspPoses* result,
                          std::string* error) {
  if (!result) {
    if (error) *error = "result pointer is null";
    return false;
  }
  if (brick.header.frame_id != parameters.planning_frame) {
    if (error) *error = "brick pose frame must match planning_frame";
    return false;
  }
  OrientedBrickGeometry oriented;
  std::string geometry_error;
  if (!deriveOrientedBrickGeometry(dimensions, &oriented, &geometry_error)) {
    if (error) *error = geometry_error;
    return false;
  }
  if (parameters.pre_grasp_height <= 0.0 ||
      parameters.lift_height <= 0.0 ||
      !std::isfinite(parameters.finger_pad_lower_edge_offset) ||
      parameters.finger_pad_lower_edge_offset <= 0.0 ||
      !std::isfinite(parameters.finger_pad_contact_overlap) ||
      parameters.finger_pad_contact_overlap <= 0.0 ||
      !std::isfinite(parameters.surface_clearance) ||
      parameters.surface_clearance < 0.0) {
    if (error) *error =
        "dimensions/vertical offsets must be positive and clearance non-negative";
    return false;
  }

  tf2::Quaternion brick_q;
  tf2::fromMsg(brick.pose.orientation, brick_q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(brick_q).getRPY(roll, pitch, yaw);
  if (std::abs(roll) > 0.15 || std::abs(pitch) > 0.15) {
    if (error) *error = "grasp requires a ground-supported 4DoF brick pose";
    return false;
  }
  if (!parameters.grasp_along_short_axis) yaw += M_PI_2;

  tf2::Quaternion tool_q;
  tool_q.setRPY(0.0, M_PI_2, yaw);
  tool_q.normalize();
  result->grasp = brick;
  result->grasp.header.frame_id = parameters.planning_frame;
  result->grasp.pose.orientation = tf2::toMsg(tool_q);
  // The nominal TCP is at the fingertip plane.  Insert until the lower edge
  // of the unchanged pad collision mesh overlaps the upper Brick side by the
  // requested amount.  This creates physical side contact without driving
  // the TCP or finger tips toward the support plane.
  const double top_surface_z =
      brick.pose.position.z + oriented.vertical_height / 2.0;
  result->grasp.pose.position.z = top_surface_z -
      parameters.finger_pad_lower_edge_offset -
      parameters.finger_pad_contact_overlap;
  const double support_plane_z =
      brick.pose.position.z - oriented.vertical_height / 2.0;
  if (result->grasp.pose.position.z <
      support_plane_z + parameters.surface_clearance) {
    if (error) *error = "AG95 TCP insertion violates support-plane ground clearance";
    return false;
  }
  result->pre_grasp = result->grasp;
  result->pre_grasp.pose.position.z += parameters.pre_grasp_height;
  result->lift = result->grasp;
  result->lift.pose.position.z += parameters.lift_height;
  if (error) error->clear();
  return true;
}

}  // namespace brick_pick_demo
