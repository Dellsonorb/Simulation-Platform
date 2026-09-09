/* SPDX-License-Identifier: BSD-3-Clause
 * Opt-in diagnostic snapshots only: never feeds an algorithm or changes physics.
 */
#pragma once

#include <array>
#include <atomic>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <mutex>

#include <gazebo/physics/physics.hh>

namespace gazebo {
namespace ground_dynamics {

inline const std::array<const char*, 6>& JointNames() {
  static const std::array<const char*, 6> names{{
      "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"}};
  return names;
}

inline double Missing() { return std::numeric_limits<double>::quiet_NaN(); }

// GlobalAxis and both link angular velocities are world-frame vectors.
// Gazebo's hinge GlobalAxis is unit length; retain native sign/order.
inline double RelativeAngularRate(const ignition::math::Vector3d& parent,
                                  const ignition::math::Vector3d& child,
                                  const ignition::math::Vector3d& global_axis) {
  return (child - parent).Dot(global_axis);
}

struct JointState {
  bool present{false};
  double position{Missing()}, velocity{Missing()}, force{Missing()};
};

struct BodyState {
  bool present{false};
  ignition::math::Pose3d pose;
  ignition::math::Vector3d linear, angular;
};

struct Snapshot {
  std::array<JointState, 6> joints;
  ignition::math::Vector3d wrist_axis{Missing(), Missing(), Missing()};
  double wrist_relative_rate{Missing()};
  BodyState base, wrist_parent, wrist_child, target;
};

// Kept independent of any live world so numeric/serialization behavior can be
// tested without a server. The caller serializes access. The stream's buffer
// is bounded; flush every 300 rows and at close, not on each physics sample.
class CsvWriter {
 public:
  ~CsvWriter() { Close(); }

  bool OpenFromEnvironment() {
    Close();
    const char* path = std::getenv("P450_GROUND_DYNAMICS_CSV");
    if (!path || !*path) return false;
    stream_.clear();
    stream_.open(path, std::ios::out | std::ios::trunc);
    if (!stream_) {
      gzerr << "Ground dynamics trace cannot open " << path << "\n";
      return false;
    }
    stream_.imbue(std::locale::classic());
    stream_ << std::setprecision(17);
    stream_ << "sim_time_s,iteration,phase";
    for (const auto name : JointNames()) {
      stream_ << ',' << name << "_present," << name << "_q_rad,"
              << name << "_dq_rad_s," << name << "_force_native";
    }
    stream_ << ",wrist_axis_world_x,wrist_axis_world_y,wrist_axis_world_z"
               ",wrist_relative_angular_rate_rad_s";
    for (const auto prefix : {"base", "wrist_parent", "wrist_child", "target"}) {
      stream_ << ',' << prefix << "_present";
      for (const auto suffix : {"x_m", "y_m", "z_m", "qw", "qx", "qy", "qz",
                                "vx_m_s", "vy_m_s", "vz_m_s",
                                "wx_rad_s", "wy_rad_s", "wz_rad_s"})
        stream_ << ',' << prefix << '_' << suffix;
    }
    stream_ << '\n';
    stream_.flush();
    return static_cast<bool>(stream_);
  }

  void Write(const Snapshot& state, double sim_time, unsigned int iteration,
             const char* phase) {
    if (!stream_.is_open() || !stream_) return;
    stream_ << sim_time << ',' << iteration << ',' << phase;
    for (const auto& joint : state.joints)
      stream_ << ',' << joint.present << ',' << joint.position << ','
              << joint.velocity << ',' << joint.force;
    Vector(state.wrist_axis);
    stream_ << ',' << state.wrist_relative_rate;
    for (const auto* body : {&state.base, &state.wrist_parent,
                             &state.wrist_child, &state.target}) Body(*body);
    stream_ << '\n';
    if (++pending_rows_ == 300) {
      stream_.flush();
      pending_rows_ = 0;
    }
  }

  void Close() {
    if (stream_.is_open()) {
      stream_.flush();
      stream_.close();
    }
    pending_rows_ = 0;
  }

 private:
  void Vector(const ignition::math::Vector3d& value) {
    stream_ << ',' << value.X() << ',' << value.Y() << ',' << value.Z();
  }

  void Body(const BodyState& body) {
    stream_ << ',' << body.present;
    if (!body.present) {
      for (unsigned int i = 0; i < 13; ++i) stream_ << ',' << Missing();
      return;
    }
    Vector(body.pose.Pos());
    const auto& rotation = body.pose.Rot();
    stream_ << ',' << rotation.W() << ',' << rotation.X() << ','
            << rotation.Y() << ',' << rotation.Z();
    Vector(body.linear);
    Vector(body.angular);
  }

  std::ofstream stream_;
  unsigned int pending_rows_{0};
};

class Trace {
 public:
  ~Trace() { Close(); }

  bool Open(const physics::ModelPtr& model, const physics::LinkPtr& base,
            const physics::WorldPtr& world) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!writer_.OpenFromEnvironment()) return false;
    model_ = model;
    base_ = base;
    world_ = world;
    for (unsigned int i = 0; i < joints_.size(); ++i)
      joints_[i] = model_->GetJoint(JointNames()[i]);
    enabled_.store(true);
    return true;
  }

  void Capture(const char* phase) {
    if (!enabled_.load()) return;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!enabled_.load()) return;
    Snapshot state;
    for (unsigned int i = 0; i < joints_.size(); ++i) {
      if (!joints_[i]) continue;
      state.joints[i].present = true;
      state.joints[i].position = joints_[i]->Position(0);
      state.joints[i].velocity = joints_[i]->GetVelocity(0);
      // Native GetForce is not a measured constraint/contact reaction wrench.
      state.joints[i].force = joints_[i]->GetForce(0);
    }
    state.base = ReadBody(base_);
    if (joints_[5]) {
      state.wrist_axis = joints_[5]->GlobalAxis(0);
      state.wrist_parent = ReadBody(joints_[5]->GetParent());
      state.wrist_child = ReadBody(joints_[5]->GetChild());
      if (state.wrist_parent.present && state.wrist_child.present)
        state.wrist_relative_rate = RelativeAngularRate(
            state.wrist_parent.angular, state.wrist_child.angular, state.wrist_axis);
    }
    // Diagnostic-only world query. Nothing is published or returned to control.
    // Target uses native Model getters; link rows use native Link getters.
    state.target = ReadBody(world_->ModelByName("pick_target"));
    writer_.Write(state, world_->SimTime().Double(), world_->Iterations(), phase);
  }

  void Close() {
    enabled_.store(false);
    std::lock_guard<std::mutex> lock(mutex_);
    writer_.Close();
    joints_.fill(physics::JointPtr());
    base_.reset();
    model_.reset();
    world_.reset();
  }

 private:
  template <typename Ptr>
  static BodyState ReadBody(const Ptr& entity) {
    BodyState body;
    if (entity) {
      body.present = true;
      body.pose = entity->WorldPose();
      body.linear = entity->WorldLinearVel();
      body.angular = entity->WorldAngularVel();
    }
    return body;
  }

  std::atomic<bool> enabled_{false};
  std::mutex mutex_;
  CsvWriter writer_;
  physics::ModelPtr model_;
  physics::LinkPtr base_;
  physics::WorldPtr world_;
  std::array<physics::JointPtr, 6> joints_;
};

}  // namespace ground_dynamics
}  // namespace gazebo
