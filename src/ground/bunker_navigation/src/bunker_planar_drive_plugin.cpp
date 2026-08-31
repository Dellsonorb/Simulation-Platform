#include <cmath>
#include <map>
#include <mutex>
#include <unordered_set>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/ode/ODECollision.hh>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>

namespace gazebo {
class BunkerPlanarDrive : public ModelPlugin {
 public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model;
    world_ = model_->GetWorld();
    namespace_ = sdf->Get<std::string>("robotNamespace", "/").first;
    cmd_topic_ = sdf->Get<std::string>("commandTopic", "/cmd_vel").first;
    odom_topic_ = sdf->Get<std::string>("odometryTopic", "/odom").first;
    odom_frame_ = sdf->Get<std::string>("odometryFrame", "odom").first;
    base_frame_ = sdf->Get<std::string>("robotBaseFrame", "base_link").first;
    handoff_command_topic_ = sdf->Get<std::string>(
        "handoffCommandTopic",
        "/ground_pick/mobile_manipulation/command").first;
    handoff_status_topic_ = sdf->Get<std::string>(
        "handoffStatusTopic",
        "/ground_pick/mobile_manipulation/status").first;
    rate_ = sdf->Get<double>("odometryRate", 50.0).first;
    node_.reset(new ros::NodeHandle(namespace_));
    cmd_sub_ = node_->subscribe(cmd_topic_, 1, &BunkerPlanarDrive::Command,
                                this);
    handoff_sub_ = node_->subscribe(
        handoff_command_topic_, 1, &BunkerPlanarDrive::HandoffCommand, this);
    status_pub_ = node_->advertise<std_msgs::String>(
        handoff_status_topic_, 1, true);
    pub_ = node_->advertise<nav_msgs::Odometry>(odom_topic_, 1);
    last_update_ = world_->SimTime();
    last_publish_ = last_update_;
    base_link_ = model_->GetLink(base_frame_);
    const physics::LinkPtr arm_root = model_->GetLink("shoulder_link");
    gripper_joint_ = model_->GetJoint("left_outer_knuckle_joint");
    if (!base_link_ || !arm_root || !gripper_joint_) {
      gzerr << "BUNKER planar drive cannot find base, arm, or AG95 joint\n";
      release_failed_ = true;
      PublishStatus("FAILED");
      return;
    }
    std::unordered_set<std::string> manipulator_names;
    std::vector<physics::LinkPtr> pending{arm_root};
    while (!pending.empty()) {
      const physics::LinkPtr link = pending.back();
      pending.pop_back();
      if (!link || !manipulator_names.insert(link->GetName()).second) {
        continue;
      }
      manipulator_links_.push_back(link);
      for (const auto& child : link->GetChildJointsLinks()) {
        pending.push_back(child);
      }
    }
    for (const auto& link : model_->GetLinks()) {
      const bool manipulator =
          manipulator_names.count(link->GetName()) != 0u;
      // Navigation treats the six-axis arm as a rigid payload.  The AG95
      // linkage is physical from startup so its controller establishes the
      // real open joint coordinate before the arm is released; otherwise the
      // Gazebo closed loop can wrap through multiple turns at handoff.
      const std::string link_name = link->GetName();
      const bool gripper =
          link_name.find("left_") == 0u || link_name.find("right_") == 0u;
      if (gripper) gripper_links_.push_back(link);
      link->SetKinematic(!gripper);
      link->SetGravityMode(false);
      link->SetLinearVel(ignition::math::Vector3d::Zero);
      link->SetAngularVel(ignition::math::Vector3d::Zero);
      for (const auto& collision : link->GetCollisions()) {
        const physics::ODECollisionPtr ode_collision =
            boost::dynamic_pointer_cast<physics::ODECollision>(collision);
        if (!ode_collision) {
          gzerr << "BUNKER planar drive requires ODE collision masks\n";
          release_failed_ = true;
          PublishStatus("FAILED");
          return;
        }
        original_collide_bits_[collision.get()] =
            static_cast<unsigned int>(
                dGeomGetCollideBits(ode_collision->GetCollisionId()));
        collision->SetCollideBits(0u);
      }
    }
    ROS_INFO_STREAM("[NAVIGATION_FIXED_PAYLOAD] BUNKER planar drive holds "
                    << manipulator_links_.size()
                    << " manipulator links until an anchored release");
    startup_started_ = world_->SimTime();
    PublishStatus("GRIPPER_SETTLING");
    connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&BunkerPlanarDrive::Update, this));
  }

 private:
  void Command(const geometry_msgs::TwistConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    command_ = *message;
    last_command_wall_ = ros::WallTime::now();
  }

  void HandoffCommand(const std_msgs::StringConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (message->data == "RELEASE") {
      release_requested_ = true;
    } else if (message->data == "ENABLE_GRAVITY") {
      gravity_requested_ = true;
    }
  }

  void PublishStatus(const std::string& state) {
    std_msgs::String message;
    message.data = state;
    status_pub_.publish(message);
  }

  void TryLockGripperPayload(const common::Time& now) {
    const double elapsed = (now - startup_started_).Double();
    const double position = gripper_joint_->Position(0);
    const double velocity = gripper_joint_->GetVelocity(0);
    const bool stable = elapsed >= 0.5 && std::isfinite(position) &&
        std::isfinite(velocity) && std::abs(position) <= 0.02 &&
        std::abs(velocity) <= 0.05;
    if (!stable) {
      gripper_stability_started_ = false;
      if (elapsed > 5.0) {
        release_failed_ = true;
        PublishStatus("FAILED");
        gzerr << "[STARTUP_GRIPPER_FAILED] AG95 did not settle open\n";
      }
      return;
    }
    if (!gripper_stability_started_) {
      gripper_stable_since_ = now;
      gripper_stability_started_ = true;
      return;
    }
    if ((now - gripper_stable_since_).Double() < 0.25) return;
    for (const auto& link : gripper_links_) {
      link->SetLinearVel(ignition::math::Vector3d::Zero);
      link->SetAngularVel(ignition::math::Vector3d::Zero);
      link->SetKinematic(true);
    }
    compatibility_initialized_ = true;
    PublishStatus("INITIAL_HOLD");
    ROS_INFO_STREAM("[STARTUP_GRIPPER_SETTLED] q=" << position
                    << " velocity=" << velocity
                    << "; AG95 locked as navigation payload");
  }

  void ReleaseAnchoredManipulator() {
    model_->SetLinearVel(ignition::math::Vector3d::Zero);
    model_->SetAngularVel(ignition::math::Vector3d::Zero);
    world_anchor_ = model_->CreateJoint(
        "ground_pick_world_anchor", "fixed", physics::LinkPtr(), base_link_);
    if (!world_anchor_) {
      release_failed_ = true;
      PublishStatus("FAILED");
      gzerr << "BUNKER planar drive failed to anchor the stopped base\n";
      return;
    }
    world_anchor_->Init();
    base_link_->SetKinematic(false);
    base_link_->SetGravityMode(false);
    base_link_->SetLinearVel(ignition::math::Vector3d::Zero);
    base_link_->SetAngularVel(ignition::math::Vector3d::Zero);
    for (const auto& link : manipulator_links_) {
      link->SetKinematic(false);
      link->SetGravityMode(false);
      link->SetLinearVel(ignition::math::Vector3d::Zero);
      link->SetAngularVel(ignition::math::Vector3d::Zero);
      for (const auto& collision : link->GetCollisions()) {
        const auto original = original_collide_bits_.find(collision.get());
        if (original == original_collide_bits_.end()) {
          release_failed_ = true;
          PublishStatus("FAILED");
          gzerr << "BUNKER planar drive lost an original collision mask\n";
          return;
        }
        collision->SetCollideBits(original->second);
      }
    }
    manipulator_released_ = true;
    PublishStatus("ANCHORED_RELEASED");
    ROS_INFO("BUNKER stopped base anchored; same-model manipulator released");
  }

  void ActivateManipulatorGravity() {
    for (const auto& link : manipulator_links_) {
      link->SetLinearVel(ignition::math::Vector3d::Zero);
      link->SetAngularVel(ignition::math::Vector3d::Zero);
      link->SetGravityMode(true);
    }
    manipulator_active_ = true;
    PublishStatus("ACTIVE_READY");
    ROS_INFO("BUNKER same-model manipulator controllers active with gravity");
  }

  void Update() {
    const common::Time now = world_->SimTime();
    if (!initialized_) {
      // EntityFactory applies the requested spawn pose after plugin Load().
      // Capture the final model pose on the first world update.
      pose_ = model_->WorldPose();
      yaw_ = pose_.Rot().Yaw();
      initialized_ = true;
      last_update_ = now;
      ROS_INFO("[STARTUP_JOINT_INITIALIZED] spawn-only compatibility pose "
               "accepted; waiting for physical AG95 open settle");
    }
    if (!compatibility_initialized_ && !release_failed_) {
      TryLockGripperPayload(now);
    }
    const double dt = (now - last_update_).Double();
    last_update_ = now;
    geometry_msgs::Twist command;
    ros::WallTime last_command_wall;
    bool release_requested = false;
    bool gravity_requested = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      command = command_;
      last_command_wall = last_command_wall_;
      release_requested = release_requested_;
      gravity_requested = gravity_requested_;
      release_requested_ = false;
      gravity_requested_ = false;
    }
    if (release_requested && compatibility_initialized_ &&
        !manipulator_released_ && !release_failed_) {
      ReleaseAnchoredManipulator();
    }
    if (gravity_requested && manipulator_released_ &&
        !manipulator_active_ && !release_failed_) {
      ActivateManipulatorGravity();
    }
    if ((ros::WallTime::now() - last_command_wall).toSec() > 0.5) {
      command = geometry_msgs::Twist();
    }
    if (manipulator_released_ || release_failed_) {
      command = geometry_msgs::Twist();
    }
    if (!compatibility_initialized_) {
      command = geometry_msgs::Twist();
    }
    if (dt > 0.0 && dt < 0.5) {
      pose_.Pos().X() += (command.linear.x * std::cos(yaw_) -
                          command.linear.y * std::sin(yaw_)) * dt;
      pose_.Pos().Y() += (command.linear.x * std::sin(yaw_) +
                          command.linear.y * std::cos(yaw_)) * dt;
      yaw_ += command.angular.z * dt;
      pose_.Rot() = ignition::math::Quaterniond(0.0, 0.0, yaw_);
      const bool moving = std::abs(command.linear.x) > 1e-9 ||
                          std::abs(command.linear.y) > 1e-9 ||
                          std::abs(command.angular.z) > 1e-9;
      if (moving) {
        model_->SetWorldPose(pose_);
        model_->SetLinearVel(ignition::math::Vector3d::Zero);
        model_->SetAngularVel(ignition::math::Vector3d::Zero);
      }
    }
    if ((now - last_publish_).Double() >= 1.0 / rate_) {
      last_publish_ = now;
      const ros::Time stamp = ros::Time::now();
      geometry_msgs::Quaternion quaternion;
      quaternion.x = pose_.Rot().X(); quaternion.y = pose_.Rot().Y();
      quaternion.z = pose_.Rot().Z(); quaternion.w = pose_.Rot().W();
      nav_msgs::Odometry odom;
      odom.header.stamp = stamp; odom.header.frame_id = odom_frame_;
      odom.child_frame_id = base_frame_;
      odom.pose.pose.position.x = pose_.Pos().X();
      odom.pose.pose.position.y = pose_.Pos().Y();
      odom.pose.pose.position.z = pose_.Pos().Z();
      odom.pose.pose.orientation = quaternion;
      odom.twist.twist = command;
      pub_.publish(odom);
      geometry_msgs::TransformStamped transform;
      transform.header = odom.header; transform.child_frame_id = base_frame_;
      transform.transform.translation.x = pose_.Pos().X();
      transform.transform.translation.y = pose_.Pos().Y();
      transform.transform.translation.z = pose_.Pos().Z();
      transform.transform.rotation = quaternion;
      broadcaster_.sendTransform(transform);
    }
  }

  physics::ModelPtr model_; physics::WorldPtr world_;
  ignition::math::Pose3d pose_; double yaw_{0.0}, rate_{50.0};
  bool initialized_{false};
  bool compatibility_initialized_{false};
  bool release_requested_{false};
  bool gravity_requested_{false};
  bool manipulator_released_{false};
  bool manipulator_active_{false};
  bool release_failed_{false};
  common::Time last_update_, last_publish_; geometry_msgs::Twist command_;
  ros::WallTime last_command_wall_{ros::WallTime::now()};
  std::mutex mutex_; std::unique_ptr<ros::NodeHandle> node_;
  physics::LinkPtr base_link_;
  physics::JointPtr gripper_joint_;
  physics::JointPtr world_anchor_;
  std::vector<physics::LinkPtr> manipulator_links_;
  std::vector<physics::LinkPtr> gripper_links_;
  std::map<physics::Collision*, unsigned int> original_collide_bits_;
  common::Time startup_started_, gripper_stable_since_;
  bool gripper_stability_started_{false};
  ros::Subscriber cmd_sub_;
  ros::Subscriber handoff_sub_;
  ros::Publisher status_pub_;
  ros::Publisher pub_; tf::TransformBroadcaster broadcaster_;
  event::ConnectionPtr connection_; std::string namespace_, cmd_topic_, odom_topic_;
  std::string odom_frame_, base_frame_;
  std::string handoff_command_topic_, handoff_status_topic_;
};
GZ_REGISTER_MODEL_PLUGIN(BunkerPlanarDrive)
}  // namespace gazebo
