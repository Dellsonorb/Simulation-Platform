#include <brick_pick_demo/pose_frames.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace brick_pick_demo {

bool transformGraspPoses(const GraspPoses& source,
                         const std::string& target_frame,
                         tf2_ros::Buffer* buffer,
                         const ros::Duration& timeout,
                         GraspPoses* result,
                         std::string* error) {
  if (!buffer || !result || target_frame.empty()) {
    if (error) *error = "transform buffer/result/target frame is invalid";
    return false;
  }
  const bool common_header =
      !source.grasp.header.frame_id.empty() &&
      source.pre_grasp.header.frame_id == source.grasp.header.frame_id &&
      source.lift.header.frame_id == source.grasp.header.frame_id &&
      source.pre_grasp.header.stamp == source.grasp.header.stamp &&
      source.lift.header.stamp == source.grasp.header.stamp;
  if (!common_header) {
    if (error) *error = "grasp poses do not share one measurement frame/stamp";
    return false;
  }
  try {
    result->grasp = buffer->transform(source.grasp, target_frame, timeout);
    result->pre_grasp =
        buffer->transform(source.pre_grasp, target_frame, timeout);
    result->lift = buffer->transform(source.lift, target_frame, timeout);
  } catch (const tf2::TransformException& exception) {
    if (error) *error = exception.what();
    return false;
  }
  if (error) error->clear();
  return true;
}

double posePositionError(const geometry_msgs::Pose& actual,
                         const geometry_msgs::Pose& target) {
  const double dx = actual.position.x - target.position.x;
  const double dy = actual.position.y - target.position.y;
  const double dz = actual.position.z - target.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double poseOrientationError(const geometry_msgs::Pose& actual,
                            const geometry_msgs::Pose& target) {
  const double actual_norm = std::sqrt(
      actual.orientation.x * actual.orientation.x +
      actual.orientation.y * actual.orientation.y +
      actual.orientation.z * actual.orientation.z +
      actual.orientation.w * actual.orientation.w);
  const double target_norm = std::sqrt(
      target.orientation.x * target.orientation.x +
      target.orientation.y * target.orientation.y +
      target.orientation.z * target.orientation.z +
      target.orientation.w * target.orientation.w);
  if (!std::isfinite(actual_norm) || !std::isfinite(target_norm) ||
      actual_norm < 1e-12 || target_norm < 1e-12) {
    return std::numeric_limits<double>::infinity();
  }
  const double dot =
      (actual.orientation.x * target.orientation.x +
       actual.orientation.y * target.orientation.y +
       actual.orientation.z * target.orientation.z +
       actual.orientation.w * target.orientation.w) /
      (actual_norm * target_norm);
  return 2.0 * std::acos(std::min(1.0, std::abs(dot)));
}

bool verifiedLift(double initial_height, double final_height,
                  double minimum_lift, double* measured_lift) {
  if (!measured_lift || !std::isfinite(initial_height) ||
      !std::isfinite(final_height) || !std::isfinite(minimum_lift) ||
      minimum_lift <= 0.0) {
    return false;
  }
  *measured_lift = final_height - initial_height;
  return std::isfinite(*measured_lift) && *measured_lift >= minimum_lift;
}

}  // namespace brick_pick_demo
