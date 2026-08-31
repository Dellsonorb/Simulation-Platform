#include <bunker_aubo_gazebo/attachment_guard.h>

#include <gazebo/common/Events.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <sstream>

namespace gazebo {

class GuardedBrickAttachment : public ModelPlugin {
 public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model;
    world_ = model_->GetWorld();
    parent_link_name_ = read(sdf, "parentLink", "wrist_3_link");
    brick_model_name_ = read(sdf, "brickModel", "brick");
    brick_link_name_ = read(sdf, "brickLink", "brick_link");
    gripper_joint_name_ = read(sdf, "gripperJoint", "left_outer_knuckle_joint");
    left_pad_link_name_ = read(sdf, "leftFingerPadLink", "left_finger_pad");
    right_pad_link_name_ = read(sdf, "rightFingerPadLink", "right_finger_pad");
    service_name_ = read(sdf, "serviceName", "/simulation/brick_attachment");
    closed_threshold_ = readDouble(sdf, "closedThreshold", 0.50);
    open_threshold_ = readDouble(sdf, "openThreshold", 0.05);
    required_opening_ = readDouble(sdf, "requiredOpening", 0.115);
    maximum_opening_ = readDouble(sdf, "maximumOpening", 0.0952);
    maximum_penetration_ = readDouble(sdf, "maximumPenetration", 0.003);
    distance_threshold_ = readDouble(sdf, "distanceThreshold", 0.35);
    region_half_width_ = readDouble(sdf, "regionHalfWidth", 0.18);
    nh_.reset(new ros::NodeHandle());
    service_ = nh_->advertiseService(service_name_,
        &GuardedBrickAttachment::request, this);
    update_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&GuardedBrickAttachment::update, this));
    contact_manager_ = world_->Physics()->GetContactManager();
    if (contact_manager_) contact_manager_->SetNeverDropContacts(true);
    ROS_INFO_STREAM("Guarded simulation-only brick attachment ready at " << service_name_);
  }

 private:
  static std::string read(const sdf::ElementPtr& sdf, const std::string& key,
                          const std::string& fallback) {
    return sdf->HasElement(key) ? sdf->Get<std::string>(key) : fallback;
  }
  static double readDouble(const sdf::ElementPtr& sdf, const std::string& key,
                           double fallback) {
    return sdf->HasElement(key) ? sdf->Get<double>(key) : fallback;
  }

  bool request(std_srvs::SetBool::Request& request,
               std_srvs::SetBool::Response& response) {
    std::unique_lock<std::mutex> lock(mutex_);
    pending_ = true;
    requested_attachment_ = request.data;
    result_ready_ = false;
    if (!condition_.wait_for(lock, std::chrono::seconds(3),
                             [this] { return result_ready_; })) {
      response.success = false;
      response.message = "Gazebo update timeout";
      return true;
    }
    response.success = result_success_;
    response.message = result_message_;
    return true;
  }

  void finish(bool success, const std::string& message) {
    result_success_ = success;
    result_message_ = message;
    result_ready_ = true;
    pending_ = false;
    condition_.notify_all();
  }

  void update() {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto gripper_joint = model_->GetJoint(gripper_joint_name_);
    if (gripper_joint && gripper_joint->Position(0) <= open_threshold_) {
      observed_preopen_ = true;
    }
    if (!pending_) return;
    if (!requested_attachment_) {
      if (fixed_joint_) {
        fixed_joint_->Detach();
        fixed_joint_.reset();
      }
      observed_preopen_ = false;
      finish(true, "brick detached");
      return;
    }
    if (fixed_joint_) {
      finish(true, "brick already attached");
      return;
    }
    const auto parent = model_->GetLink(parent_link_name_);
    const auto left_pad = model_->GetLink(left_pad_link_name_);
    const auto right_pad = model_->GetLink(right_pad_link_name_);
    const auto brick_model = world_->ModelByName(brick_model_name_);
    const auto brick = brick_model ? brick_model->GetLink(brick_link_name_) : physics::LinkPtr();
    if (!gripper_joint || !parent || !left_pad || !right_pad || !brick) {
      finish(false, "required gripper or brick entity is missing");
      return;
    }
    bunker_aubo_gazebo::AttachmentEvidence evidence;
    evidence.geometry_feasible = required_opening_ <= maximum_opening_;
    evidence.preopened = observed_preopen_;
    evidence.gripper_closed =
        gripper_joint->Position(0) >= closed_threshold_;
    const ignition::math::Vector3d left_position = left_pad->WorldPose().Pos();
    const ignition::math::Vector3d right_position = right_pad->WorldPose().Pos();
    const ignition::math::Vector3d pad_axis = right_position - left_position;
    const double squared_pad_separation = pad_axis.SquaredLength();
    evidence.pad_center_separation = std::sqrt(squared_pad_separation);
    if (std::isfinite(squared_pad_separation) &&
        squared_pad_separation > 1e-10) {
      evidence.brick_axis_fraction =
          (brick->WorldPose().Pos() - left_position).Dot(pad_axis) /
          squared_pad_separation;
      evidence.brick_bracketed =
          std::isfinite(evidence.brick_axis_fraction) &&
          evidence.brick_axis_fraction >= 0.05 &&
          evidence.brick_axis_fraction <= 0.95;
    }
    if (!evidence.geometry_feasible) {
      const bunker_aubo_gazebo::AttachmentDecision decision =
          bunker_aubo_gazebo::decideAttachment(evidence, maximum_penetration_);
      std::ostringstream audit;
      audit << decision.reason << "; required_opening=" << required_opening_
            << " maximum_opening=" << maximum_opening_
            << " preopened=" << evidence.preopened
            << " gripper_q=" << gripper_joint->Position(0);
      finish(false, audit.str());
      return;
    }
    const ignition::math::Vector3d delta = brick->WorldPose().Pos() - parent->WorldPose().Pos();
    const ignition::math::Vector3d local = parent->WorldPose().Rot().RotateVectorReverse(delta);
    // wrist_3_link's local Z is the tool approach axis in the AUBO model;
    // constrain the two lateral axes and the total reach independently.
    if (delta.Length() > distance_threshold_ || std::abs(local.X()) > region_half_width_ ||
        std::abs(local.Y()) > region_half_width_) {
      std::ostringstream message;
      message << "brick is outside guarded grasp region: distance=" << delta.Length()
              << " local=[" << local.X() << "," << local.Y() << ","
              << local.Z() << "]";
      finish(false, message.str());
      return;
    }
    if (contact_manager_) {
      ignition::math::Vector3d left_normal_sum(0.0, 0.0, 0.0);
      ignition::math::Vector3d right_normal_sum(0.0, 0.0, 0.0);
      const unsigned int count = contact_manager_->GetContactCount();
      for (unsigned int index = 0; index < count; ++index) {
        const physics::Contact* contact = contact_manager_->GetContact(index);
        if (!contact || !contact->collision1 || !contact->collision2) continue;
        const physics::LinkPtr link1 = contact->collision1->GetLink();
        const physics::LinkPtr link2 = contact->collision2->GetLink();
        if (!link1 || !link2) continue;
        const std::string first = link1->GetName();
        const std::string second = link2->GetName();
        const bool brick_first = first == brick_link_name_;
        const bool brick_second = second == brick_link_name_;
        if (!brick_first && !brick_second) continue;
        const std::string other = brick_first ? second : first;
        for (int point = 0; point < contact->count; ++point) {
          ignition::math::Vector3d normal = contact->normals[point];
          // Normalize collision ordering so both accumulated vectors point
          // consistently from the Brick toward the contacting pad.
          if (!brick_first) normal = -normal;
          if (other == left_pad_link_name_) {
            evidence.left_contact = true;
            ++evidence.left_contact_count;
            left_normal_sum += normal;
          }
          if (other == right_pad_link_name_) {
            evidence.right_contact = true;
            ++evidence.right_contact_count;
            right_normal_sum += normal;
          }
          evidence.maximum_penetration = std::max(
              evidence.maximum_penetration, contact->depths[point]);
        }
      }
      if (evidence.left_contact_count > 0 &&
          evidence.right_contact_count > 0 &&
          left_normal_sum.Length() > 1e-9 &&
          right_normal_sum.Length() > 1e-9) {
        left_normal_sum.Normalize();
        right_normal_sum.Normalize();
        evidence.opposed_contact_normals =
            left_normal_sum.Dot(right_normal_sum) < -0.5;
      }
    }
    const bunker_aubo_gazebo::AttachmentDecision decision =
        bunker_aubo_gazebo::decideAttachment(evidence, maximum_penetration_);
    std::ostringstream audit;
    audit << decision.reason << "; required_opening=" << required_opening_
          << " maximum_opening=" << maximum_opening_
          << " preopened=" << evidence.preopened
          << " gripper_q=" << gripper_joint->Position(0)
          << " left_contact=" << evidence.left_contact
          << " right_contact=" << evidence.right_contact
          << " left_contact_count=" << evidence.left_contact_count
          << " right_contact_count=" << evidence.right_contact_count
          << " brick_bracketed=" << evidence.brick_bracketed
          << " opposed_normals=" << evidence.opposed_contact_normals
          << " pad_center_separation=" << evidence.pad_center_separation
          << " brick_axis_fraction=" << evidence.brick_axis_fraction
          << " max_penetration=" << evidence.maximum_penetration;
    if (!decision.allowed) {
      finish(false, audit.str());
      return;
    }
    fixed_joint_ = world_->Physics()->CreateJoint("fixed", model_);
    fixed_joint_->Load(parent, brick, ignition::math::Pose3d());
    fixed_joint_->Init();
    finish(true, audit.str());
  }

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  physics::JointPtr fixed_joint_;
  event::ConnectionPtr update_;
  std::unique_ptr<ros::NodeHandle> nh_;
  ros::ServiceServer service_;
  physics::ContactManager* contact_manager_{nullptr};
  std::string parent_link_name_, brick_model_name_, brick_link_name_;
  std::string left_pad_link_name_, right_pad_link_name_;
  std::string gripper_joint_name_, service_name_;
  double closed_threshold_{0.5}, distance_threshold_{0.35}, region_half_width_{0.18};
  double open_threshold_{0.05}, required_opening_{0.115};
  double maximum_opening_{0.0952}, maximum_penetration_{0.003};
  std::mutex mutex_;
  std::condition_variable condition_;
  bool pending_{false}, requested_attachment_{false}, result_ready_{false};
  bool observed_preopen_{false};
  bool result_success_{false};
  std::string result_message_;
};

GZ_REGISTER_MODEL_PLUGIN(GuardedBrickAttachment)
}  // namespace gazebo
