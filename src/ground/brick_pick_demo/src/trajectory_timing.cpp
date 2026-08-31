#include <brick_pick_demo/trajectory_timing.h>

#include <algorithm>
#include <cmath>

namespace brick_pick_demo {
namespace {

bool finiteVector(const std::vector<double>& values) {
  for (const double value : values) {
    if (!std::isfinite(value)) return false;
  }
  return true;
}

bool fail(const std::string& message, std::string* error) {
  if (error) *error = message;
  return false;
}

}  // namespace

bool retimeForCartesianSpeed(trajectory_msgs::JointTrajectory* trajectory,
                             double cartesian_distance,
                             double cartesian_speed,
                             std::string* error) {
  if (error) error->clear();
  if (!trajectory) return fail("trajectory is null", error);
  if (!std::isfinite(cartesian_distance) || cartesian_distance < 0.0 ||
      !std::isfinite(cartesian_speed) || cartesian_speed <= 0.0) {
    return fail("Cartesian distance and speed are invalid", error);
  }
  if (trajectory->joint_names.empty() || trajectory->points.size() < 2) {
    return fail("trajectory must contain joints and at least two points", error);
  }

  const std::size_t joint_count = trajectory->joint_names.size();
  double previous_time = -1.0;
  for (const auto& point : trajectory->points) {
    const double time = point.time_from_start.toSec();
    if (!std::isfinite(time) || time <= previous_time ||
        point.positions.size() != joint_count ||
        (!point.velocities.empty() && point.velocities.size() != joint_count) ||
        (!point.accelerations.empty() &&
         point.accelerations.size() != joint_count) ||
        !finiteVector(point.positions) || !finiteVector(point.velocities) ||
        !finiteVector(point.accelerations)) {
      return fail("trajectory contains malformed or non-finite points", error);
    }
    previous_time = time;
  }

  const double original_duration = trajectory->points.back().time_from_start.toSec();
  if (original_duration <= 0.0) {
    return fail("trajectory duration must be positive", error);
  }
  const double required_duration = cartesian_distance / cartesian_speed;
  const double scale = std::max(1.0, required_duration / original_duration);
  for (auto& point : trajectory->points) {
    point.time_from_start = ros::Duration(point.time_from_start.toSec() * scale);
    for (double& velocity : point.velocities) velocity /= scale;
    for (double& acceleration : point.accelerations) {
      acceleration /= scale * scale;
    }
  }
  trajectory->points.back().velocities.assign(joint_count, 0.0);
  trajectory->points.back().accelerations.assign(joint_count, 0.0);
  return true;
}

}  // namespace brick_pick_demo
