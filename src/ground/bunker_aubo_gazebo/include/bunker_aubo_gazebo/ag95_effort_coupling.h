#pragma once

#include <string>

namespace bunker_aubo_gazebo {

struct EffortCouplingParameters {
  double position_gain{5.0};
  double velocity_gain{0.1};
  double maximum_effort{50.0};
};

struct EffortCouplingResult {
  bool valid{false};
  double effort{0.0};
  std::string reason;
};

EffortCouplingResult calculateEffortCoupling(
    double master_position, double master_velocity, double passive_position,
    double passive_velocity, const EffortCouplingParameters& parameters);

}  // namespace bunker_aubo_gazebo
