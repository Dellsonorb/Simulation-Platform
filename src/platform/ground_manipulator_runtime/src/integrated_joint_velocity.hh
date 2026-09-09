/* SPDX-License-Identifier: BSD-3-Clause */
#pragma once

#include <cmath>
#include <cstring>

namespace ground_manipulator_runtime {

inline bool IntegratedVelocityEnabled(const char* value) {
  return value && std::strcmp(value, "1") == 0;
}

// Interval-average velocity of integrated encoder position, not instantaneous
// native ODE dq. QuickStep integrates ERP-corrected velocities into poses, then
// removes their ERP component from native velocity feedback. Use every valid
// consecutive read: no filtering, clipping, deadband or quiet-sample selection.
// Position is already unwrapped by DefaultRobotHWSim::readSim.
class IntegratedJointVelocity {
 public:
  bool Update(double time, double position, double& velocity) {
    if (!std::isfinite(time) || !std::isfinite(position) || !std::isfinite(velocity)) {
      // Never conceal an invalid native physics sample with a finite estimate.
      primed_ = false;
      return false;
    }
    const double elapsed = time - previous_time_;
    const bool valid_interval = primed_ && std::isfinite(elapsed) && elapsed > 0.;
    const double delta = position - previous_position_;
    previous_time_ = time;
    previous_position_ = position;
    primed_ = true;
    if (!valid_interval) return false;
    const double integrated_velocity = delta / elapsed;
    if (!std::isfinite(integrated_velocity)) return false;
    velocity = integrated_velocity;
    return true;
  }

 private:
  bool primed_{false};
  double previous_time_{0.}, previous_position_{0.};
};

}  // namespace ground_manipulator_runtime
