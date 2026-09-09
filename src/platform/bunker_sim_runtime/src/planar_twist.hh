#pragma once
#include <ignition/math/Vector3.hh>

namespace gazebo {
// ROS Twist is at base_link; ODE SetLinearVel is at the body's center of mass.
inline ignition::math::Vector3d CoGVelocity(
    const ignition::math::Vector3d& origin_velocity,
    const ignition::math::Vector3d& angular_velocity,
    const ignition::math::Vector3d& origin_to_cog) {
  return origin_velocity + angular_velocity.Cross(origin_to_cog);
}
}  // namespace gazebo
