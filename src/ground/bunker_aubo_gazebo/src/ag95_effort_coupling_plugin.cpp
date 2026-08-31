#include <bunker_aubo_gazebo/ag95_effort_coupling.h>

#include <gazebo/common/Events.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>

#include <functional>
#include <string>
#include <vector>

namespace gazebo {

class Ag95EffortCouplingPlugin : public ModelPlugin {
 public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model;
    const std::string master_name = read(
        sdf, "masterJoint", "left_outer_knuckle_joint");
    master_ = model_->GetJoint(master_name);
    if (!master_) {
      gzerr << "AG95 effort coupling missing master joint " << master_name
            << "\n";
      return;
    }
    if (sdf->HasElement("positionGain"))
      parameters_.position_gain = sdf->Get<double>("positionGain");
    if (sdf->HasElement("velocityGain"))
      parameters_.velocity_gain = sdf->Get<double>("velocityGain");
    if (sdf->HasElement("maximumEffort"))
      parameters_.maximum_effort = sdf->Get<double>("maximumEffort");

    if (sdf->HasElement("passiveJoint")) {
      sdf::ElementPtr element = sdf->GetElement("passiveJoint");
      while (element) {
        const std::string name = element->Get<std::string>();
        physics::JointPtr joint = model_->GetJoint(name);
        if (!joint) {
          gzerr << "AG95 effort coupling missing passive joint " << name
                << "\n";
          passive_.clear();
          return;
        }
        passive_.push_back(joint);
        element = element->GetNextElement("passiveJoint");
      }
    }
    if (passive_.empty()) {
      gzerr << "AG95 effort coupling has no passive joints\n";
      return;
    }
    const bunker_aubo_gazebo::EffortCouplingResult calibration =
        bunker_aubo_gazebo::calculateEffortCoupling(
            0.0, 0.0, 0.0, 0.0, parameters_);
    if (!calibration.valid) {
      gzerr << "AG95 effort coupling rejected calibration: "
            << calibration.reason << "\n";
      passive_.clear();
      return;
    }
    update_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&Ag95EffortCouplingPlugin::update, this));
    ROS_INFO("AG95 passive linkage uses bounded effort coupling only: "
             "kp=%.3f kd=%.3f max_effort=%.3f joints=%zu",
             parameters_.position_gain, parameters_.velocity_gain,
             parameters_.maximum_effort, passive_.size());
  }

 private:
  static std::string read(const sdf::ElementPtr& sdf, const std::string& key,
                          const std::string& fallback) {
    return sdf->HasElement(key) ? sdf->Get<std::string>(key) : fallback;
  }

  void update() {
    const double master_position = master_->Position(0);
    const double master_velocity = master_->GetVelocity(0);
    for (const physics::JointPtr& joint : passive_) {
      const bunker_aubo_gazebo::EffortCouplingResult result =
          bunker_aubo_gazebo::calculateEffortCoupling(
              master_position, master_velocity, joint->Position(0),
              joint->GetVelocity(0), parameters_);
      if (!result.valid) {
        ROS_ERROR_THROTTLE(1.0, "AG95 effort coupling fail-closed: %s",
                           result.reason.c_str());
        joint->SetForce(0, 0.0);
        continue;
      }
      joint->SetForce(0, result.effort);
    }
  }

  physics::ModelPtr model_;
  physics::JointPtr master_;
  std::vector<physics::JointPtr> passive_;
  event::ConnectionPtr update_;
  bunker_aubo_gazebo::EffortCouplingParameters parameters_;
};

GZ_REGISTER_MODEL_PLUGIN(Ag95EffortCouplingPlugin)
}  // namespace gazebo
