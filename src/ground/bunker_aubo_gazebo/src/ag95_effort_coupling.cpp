#include <bunker_aubo_gazebo/ag95_effort_coupling.h>

#include <algorithm>
#include <cmath>

namespace bunker_aubo_gazebo {

EffortCouplingResult calculateEffortCoupling(
    double master_position, double master_velocity, double passive_position,
    double passive_velocity, const EffortCouplingParameters& parameters) {
  EffortCouplingResult result;
  const double values[] = {
      master_position, master_velocity, passive_position, passive_velocity,
      parameters.position_gain, parameters.velocity_gain,
      parameters.maximum_effort};
  for (double value : values) {
    if (!std::isfinite(value)) {
      result.reason = "NONFINITE_AG95_COUPLING_STATE";
      return result;
    }
  }
  constexpr double kPhysicalMaximumEffort = 50.0;
  if (parameters.position_gain < 0.0 || parameters.velocity_gain < 0.0 ||
      parameters.maximum_effort <= 0.0 ||
      parameters.maximum_effort > kPhysicalMaximumEffort) {
    result.reason = "INVALID_AG95_COUPLING_CALIBRATION";
    return result;
  }
  const double raw = parameters.position_gain *
                         (master_position - passive_position) +
                     parameters.velocity_gain *
                         (master_velocity - passive_velocity);
  result.effort = std::max(-parameters.maximum_effort,
                           std::min(parameters.maximum_effort, raw));
  result.valid = true;
  return result;
}

}  // namespace bunker_aubo_gazebo
