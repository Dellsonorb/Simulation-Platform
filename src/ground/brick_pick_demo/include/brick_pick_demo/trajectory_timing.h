#pragma once

#include <string>

#include <trajectory_msgs/JointTrajectory.h>

namespace brick_pick_demo {

bool retimeForCartesianSpeed(trajectory_msgs::JointTrajectory* trajectory,
                             double cartesian_distance,
                             double cartesian_speed,
                             std::string* error);

}  // namespace brick_pick_demo
