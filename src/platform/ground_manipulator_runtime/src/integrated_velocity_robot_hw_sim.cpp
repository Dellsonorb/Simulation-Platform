/* SPDX-License-Identifier: BSD-3-Clause */
#include <cstdlib>
#include <vector>

#include <gazebo_ros_control/default_robot_hw_sim.h>
#include <pluginlib/class_list_macros.h>

#include "integrated_joint_velocity.hh"

namespace ground_manipulator_runtime {

class IntegratedVelocityRobotHWSim final
    : public gazebo_ros_control::DefaultRobotHWSim {
 public:
  bool initSim(const std::string& robot_namespace, ros::NodeHandle model_nh,
               gazebo::physics::ModelPtr parent_model,
               const urdf::Model* const urdf_model,
               std::vector<transmission_interface::TransmissionInfo> transmissions) override {
    if (!DefaultRobotHWSim::initSim(robot_namespace, model_nh, parent_model,
                                  urdf_model, transmissions)) return false;
    integrated_velocity_ = IntegratedVelocityEnabled(
        std::getenv("P450_GROUND_INTEGRATED_VELOCITY"));
    if (integrated_velocity_) samples_.resize(n_dof_);
    ROS_INFO_STREAM("Ground joint velocity feedback mode: " <<
        (integrated_velocity_ ? "integrated_position_interval_average" : "stock_native") <<
        "; stock writeSim, joint limits and PID gains retained");
    return true;
  }

  void readSim(ros::Time time, ros::Duration period) override {
    DefaultRobotHWSim::readSim(time, period);
    if (!integrated_velocity_) return;
    for (unsigned int joint = 0; joint < n_dof_; ++joint) {
      // Registered handles point into joint_velocity_: modify elements only.
      // First/invalid/reset samples retain the stock native value and re-prime.
      samples_[joint].Update(time.toSec(), joint_position_[joint], joint_velocity_[joint]);
    }
  }

 private:
  bool integrated_velocity_{false};
  std::vector<IntegratedJointVelocity> samples_;
};

}  // namespace ground_manipulator_runtime

PLUGINLIB_EXPORT_CLASS(ground_manipulator_runtime::IntegratedVelocityRobotHWSim,
                       gazebo_ros_control::RobotHWSim)
