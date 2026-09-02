/* SPDX-License-Identifier: BSD-3-Clause
 * A small ROS 1 / Gazebo planar velocity plugin for the BUNKER runtime.
 * It deliberately owns and joins its callback thread before Gazebo destroys
 * the ROS callback queue.
 */

#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <boost/bind/bind.hpp>
#include <bunker_msgs/BunkerStatus.h>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

namespace gazebo {
namespace {

template <typename T>
T ReadParameter(const sdf::ElementPtr& sdf, const std::string& name,
                const T& fallback) {
  if (!sdf->HasElement(name)) {
    return fallback;
  }
  return sdf->GetElement(name)->Get<T>();
}

std::string StripLeadingSlash(const std::string& value) {
  const std::size_t first = value.find_first_not_of('/');
  return first == std::string::npos ? std::string() : value.substr(first);
}

}  // namespace

class BunkerPlanarMovePlugin final : public ModelPlugin {
 public:
  BunkerPlanarMovePlugin() = default;

  ~BunkerPlanarMovePlugin() override { Shutdown(); }

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    if (!ros::isInitialized()) {
      gzerr << "BUNKER planar move requires gazebo_ros_api_plugin\n";
      return;
    }

    model_ = std::move(model);
    world_ = model_->GetWorld();
    robot_namespace_ = ReadParameter<std::string>(
        sdf, "robotNamespace", "/ground");
    command_topic_ = ReadParameter<std::string>(
        sdf, "commandTopic", "cmd_vel");
    odometry_topic_ = ReadParameter<std::string>(
        sdf, "odometryTopic", "odom");
    status_topic_ = ReadParameter<std::string>(
        sdf, "statusTopic", "bunker_status");
    odometry_frame_ = ReadParameter<std::string>(
        sdf, "odometryFrame", "odom");
    robot_base_frame_ = ReadParameter<std::string>(
        sdf, "robotBaseFrame", "base_link");
    base_link_ = model_->GetLink(StripLeadingSlash(robot_base_frame_));
    if (!base_link_) {
      gzerr << "BUNKER planar move cannot resolve base link "
            << robot_base_frame_ << "\n";
      return;
    }
    initial_base_pose_ = base_link_->WorldPose();
    odometry_rate_ = ReadParameter<double>(sdf, "odometryRate", 50.0);
    command_timeout_ = ReadParameter<double>(sdf, "cmdTimeout", 0.5);
    if (odometry_rate_ <= 0.0 || command_timeout_ < 0.0) {
      gzerr << "BUNKER planar move received invalid timing parameters\n";
      return;
    }

    node_.reset(new ros::NodeHandle(robot_namespace_));
    node_->param<std::string>("tf_prefix", tf_prefix_, std::string());
    tf_broadcaster_.reset(new tf2_ros::TransformBroadcaster());

    ros::SubscribeOptions options =
        ros::SubscribeOptions::create<geometry_msgs::Twist>(
            command_topic_, 1,
            boost::bind(&BunkerPlanarMovePlugin::CommandCallback, this,
                        boost::placeholders::_1),
            ros::VoidPtr(), &callback_queue_);
    command_subscriber_ = node_->subscribe(options);
    odometry_publisher_ = node_->advertise<nav_msgs::Odometry>(
        odometry_topic_, 1);
    status_publisher_ = node_->advertise<bunker_msgs::BunkerStatus>(
        status_topic_, 1);

    last_command_time_ = ros::Time::now();
    last_update_time_ = world_->SimTime();
    last_publish_time_ = world_->SimTime();
    running_.store(true);
    callback_thread_ = std::thread(
        &BunkerPlanarMovePlugin::RunCallbackQueue, this);
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&BunkerPlanarMovePlugin::Update, this));
    ROS_INFO_STREAM("BUNKER planar move ready on "
                    << node_->resolveName(command_topic_));
  }

 private:
  void Shutdown() {
    update_connection_.reset();
    running_.store(false);
    callback_queue_.disable();
    command_subscriber_.shutdown();
    if (node_) {
      node_->shutdown();
    }
    if (callback_thread_.joinable()) {
      callback_thread_.join();
    }
    odometry_publisher_.shutdown();
    status_publisher_.shutdown();
    tf_broadcaster_.reset();
    node_.reset();
    base_link_.reset();
  }

  void RunCallbackQueue() {
    const ros::WallDuration timeout(0.01);
    while (running_.load()) {
      callback_queue_.callAvailable(timeout);
    }
  }

  void CommandCallback(const geometry_msgs::TwistConstPtr& message) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_ = *message;
    last_command_time_ = ros::Time::now();
  }

  std::string ResolveFrame(const std::string& frame) const {
    const std::string clean_frame = StripLeadingSlash(frame);
    const std::string clean_prefix = StripLeadingSlash(tf_prefix_);
    if (clean_prefix.empty()) {
      return clean_frame;
    }
    if (clean_frame == clean_prefix ||
        clean_frame.compare(0, clean_prefix.size() + 1,
                            clean_prefix + "/") == 0) {
      return clean_frame;
    }
    return clean_prefix + "/" + clean_frame;
  }

  geometry_msgs::Twist CurrentCommand() {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if ((ros::Time::now() - last_command_time_).toSec() > command_timeout_) {
      return geometry_msgs::Twist();
    }
    return command_;
  }

  void Update() {
    if (!running_.load()) {
      return;
    }
    const geometry_msgs::Twist command = CurrentCommand();
    const ignition::math::Pose3d base_pose = base_link_->WorldPose();
    const double yaw = base_pose.Rot().Yaw();
    base_link_->SetLinearVel(ignition::math::Vector3d(
        command.linear.x * std::cos(yaw) - command.linear.y * std::sin(yaw),
        command.linear.y * std::cos(yaw) + command.linear.x * std::sin(yaw),
        0.0));
    base_link_->SetAngularVel(ignition::math::Vector3d(
        0.0, 0.0, command.angular.z));

    const common::Time now = world_->SimTime();
    const double elapsed = (now - last_update_time_).Double();
    last_update_time_ = now;
    const ignition::math::Vector3d world_linear = base_link_->WorldLinearVel();
    const ignition::math::Vector3d world_angular = base_link_->WorldAngularVel();
    const double linear_velocity =
        world_linear.X() * std::cos(yaw) + world_linear.Y() * std::sin(yaw);
    const double angular_velocity = world_angular.Z();
    if (elapsed < 0.0) {
      initial_base_pose_ = base_pose;
    }
    const ignition::math::Pose3d relative_pose =
        initial_base_pose_.Inverse() * base_pose;
    odometry_x_ = relative_pose.Pos().X();
    odometry_y_ = relative_pose.Pos().Y();
    odometry_yaw_ = relative_pose.Rot().Yaw();
    if ((now - last_publish_time_).Double() < 1.0 / odometry_rate_) {
      return;
    }
    last_publish_time_ = now;
    PublishOdometry(linear_velocity, angular_velocity);
  }

  void PublishOdometry(double linear_velocity, double angular_velocity) {
    const ros::Time stamp = ros::Time::now();
    const std::string odometry_frame = ResolveFrame(odometry_frame_);
    const std::string base_frame = ResolveFrame(robot_base_frame_);

    nav_msgs::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = odometry_frame;
    odometry.child_frame_id = base_frame;
    odometry.pose.pose.position.x = odometry_x_;
    odometry.pose.pose.position.y = odometry_y_;
    odometry.pose.pose.position.z = 0.0;
    odometry.pose.pose.orientation.z = std::sin(odometry_yaw_ / 2.0);
    odometry.pose.pose.orientation.w = std::cos(odometry_yaw_ / 2.0);
    odometry.twist.twist.linear.x = linear_velocity;
    odometry.twist.twist.angular.z = angular_velocity;
    odometry.pose.covariance[0] = 1e-5;
    odometry.pose.covariance[7] = 1e-5;
    odometry.pose.covariance[35] = 1e-3;
    odometry_publisher_.publish(odometry);

    geometry_msgs::TransformStamped transform;
    transform.header = odometry.header;
    transform.child_frame_id = base_frame;
    transform.transform.translation.x = odometry_x_;
    transform.transform.translation.y = odometry_y_;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = odometry.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);

    bunker_msgs::BunkerStatus status;
    status.header.stamp = stamp;
    status.header.frame_id = base_frame;
    status.linear_velocity = linear_velocity;
    status.angular_velocity = angular_velocity;
    status.base_state = 0;
    status.control_mode = 0;
    status.fault_code = 0;
    status.battery_voltage = 0.0;
    status_publisher_.publish(status);
  }

  physics::ModelPtr model_;
  physics::LinkPtr base_link_;
  physics::WorldPtr world_;
  ignition::math::Pose3d initial_base_pose_;
  event::ConnectionPtr update_connection_;
  std::unique_ptr<ros::NodeHandle> node_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  ros::Subscriber command_subscriber_;
  ros::Publisher odometry_publisher_;
  ros::Publisher status_publisher_;
  ros::CallbackQueue callback_queue_;
  std::thread callback_thread_;
  std::atomic<bool> running_{false};
  std::mutex command_mutex_;
  geometry_msgs::Twist command_;
  ros::Time last_command_time_;
  common::Time last_publish_time_;
  common::Time last_update_time_;
  std::string robot_namespace_;
  std::string command_topic_;
  std::string odometry_topic_;
  std::string status_topic_;
  std::string odometry_frame_;
  std::string robot_base_frame_;
  std::string tf_prefix_;
  double odometry_rate_{50.0};
  double command_timeout_{0.5};
  double odometry_x_{0.0};
  double odometry_y_{0.0};
  double odometry_yaw_{0.0};
};

GZ_REGISTER_MODEL_PLUGIN(BunkerPlanarMovePlugin)
}  // namespace gazebo
