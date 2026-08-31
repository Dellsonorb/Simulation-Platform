#include <brick_pick_demo/grasp_geometry.h>
#include <brick_pick_demo/gripper_feasibility.h>
#include <brick_pick_demo/pose_frames.h>
#include <brick_pick_demo/trajectory_timing.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/GetModelState.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <ros/ros.h>
#include <ros/topic.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/SetBool.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <mutex>
#include <stdexcept>

namespace brick_pick_demo {

class BrickPickNode {
 public:
  BrickPickNode()
      : nh_(), pnh_("~"), tf_buffer_(ros::Duration(30.0)),
        tf_listener_(tf_buffer_), arm_("manipulator"),
        gripper_("/gripper_controller/follow_joint_trajectory", true) {
    pnh_.param("brick/length", dimensions_.length, dimensions_.length);
    pnh_.param("brick/width", dimensions_.width, dimensions_.width);
    pnh_.param("brick/height", dimensions_.height, dimensions_.height);
    pnh_.param("brick/orientation_mode", dimensions_.orientation_mode,
               dimensions_.orientation_mode);
    pnh_.param("grasp/planning_frame", parameters_.planning_frame,
               parameters_.planning_frame);
    pnh_.param("grasp/pre_grasp_height", parameters_.pre_grasp_height,
               parameters_.pre_grasp_height);
    pnh_.param("grasp/lift_height", parameters_.lift_height,
               parameters_.lift_height);
    pnh_.param("grasp/surface_clearance", parameters_.surface_clearance,
               parameters_.surface_clearance);
    pnh_.param("grasp/finger_pad_lower_edge_offset",
               parameters_.finger_pad_lower_edge_offset,
               parameters_.finger_pad_lower_edge_offset);
    pnh_.param("grasp/finger_pad_contact_overlap",
               parameters_.finger_pad_contact_overlap,
               parameters_.finger_pad_contact_overlap);
    pnh_.param("grasp/grasp_along_short_axis",
               parameters_.grasp_along_short_axis,
               parameters_.grasp_along_short_axis);
    pnh_.param("grasp/cartesian_eef_step", eef_step_, 0.005);
    pnh_.param("grasp/cartesian_min_fraction", min_fraction_, 0.95);
    pnh_.param("grasp/pose_position_tolerance",
               pose_position_tolerance_, 0.010);
    pnh_.param("grasp/pose_orientation_tolerance",
               pose_orientation_tolerance_, 0.050);
    pnh_.param("grasp/approach_speed", approach_speed_, 0.03);
    pnh_.param("gripper/open_position", open_position_, 0.0);
    pnh_.param("gripper/open_tolerance", open_tolerance_, 0.10);
    pnh_.param("gripper/closed_position", closed_position_, 0.70);
    pnh_.param("gripper/closed_threshold", closed_threshold_, 0.20);
    pnh_.param("gripper/motion_time", gripper_motion_time_, 2.0);
    pnh_.param("gripper/settle_time", settle_time_, 0.5);
    pnh_.param("gripper/maximum_opening", maximum_opening_, 0.0952);
    pnh_.param("gripper/maximum_joint_position", maximum_joint_position_,
               0.93);
    pnh_.param("gripper/opening_margin", opening_margin_, 0.002);
    pnh_.param("simulation/use_link_attacher", use_attacher_, true);
    pnh_.param("simulation/attachment_service", attachment_service_,
               std::string("/simulation/brick_attachment"));
    pnh_.param("simulation/brick_model", brick_model_,
               std::string("brick"));
    pnh_.param("simulation/minimum_brick_lift", minimum_brick_lift_, 0.10);
    if (!std::isfinite(approach_speed_) || approach_speed_ <= 0.0) {
      throw std::runtime_error("grasp/approach_speed must be positive and finite");
    }
    // computeCartesianPath accepts bare Pose waypoints and therefore uses the
    // MoveGroup pose-reference frame.  Keep that frame equal to the robot
    // model/planning frame; source Brick poses are transformed explicitly at
    // their measurement stamp before Cartesian execution.
    arm_.setPoseReferenceFrame(arm_.getPlanningFrame());
    arm_.setEndEffectorLink("gripper_tcp_link");
    arm_.setPlannerId("RRTConnectkConfigDefault");
    arm_.setPlanningTime(10.0);
    subscriber_ = nh_.subscribe("/brick_pose", 1, &BrickPickNode::poseCallback, this);
    ROS_INFO("[WAIT_FOR_BRICK_POSE] brick_pick_node is ready");
  }

 private:
  bool readGripperPosition(double* position) {
    const auto state = ros::topic::waitForMessage<sensor_msgs::JointState>(
        "/joint_states", nh_, ros::Duration(2.0));
    if (!state) return false;
    for (std::size_t index = 0; index < state->name.size(); ++index) {
      if (state->name[index] == "left_outer_knuckle_joint") {
        *position = state->position[index];
        return true;
      }
    }
    return false;
  }

  bool commandGripper(double position, bool accept_contact_stall = false,
                      bool accept_measured_target = false) {
    if (!gripper_.waitForServer(ros::Duration(15.0))) return false;
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names.push_back("left_outer_knuckle_joint");
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.push_back(position);
    point.time_from_start = ros::Duration(gripper_motion_time_);
    goal.trajectory.points.push_back(point);
    gripper_.sendGoal(goal);
    if (!gripper_.waitForResult(ros::Duration(gripper_motion_time_ + 5.0))) return false;
    if (gripper_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      return true;
    ROS_WARN("[GRIPPER] controller returned %s", gripper_.getState().toString().c_str());
    double measured_position = 0.0;
    if (!readGripperPosition(&measured_position)) return false;
    if (accept_measured_target &&
        std::abs(measured_position - position) <= open_tolerance_) {
      ROS_WARN("[OPEN_GRIPPER] accepting measured position %.3f rad within %.3f rad",
               measured_position, open_tolerance_);
      return true;
    }
    if (!accept_contact_stall) return false;
    ROS_WARN("[CLOSE_GRIPPER] contact stall at %.3f rad (threshold %.3f)",
             measured_position, closed_threshold_);
    return measured_position >= closed_threshold_;
  }

  bool verifyPreOpen(double required_opening) {
    double measured_joint = 0.0;
    if (!readGripperPosition(&measured_joint)) {
      ROS_ERROR("[VERIFY_OPENING] AG95 master joint is unavailable");
      return false;
    }
    double measured_opening = 0.0;
    std::string error;
    if (!estimateJawOpening(measured_joint, maximum_opening_,
                            maximum_joint_position_, &measured_opening,
                            &error)) {
      ROS_ERROR("[VERIFY_OPENING] %s", error.c_str());
      return false;
    }
    ROS_INFO("[VERIFY_OPENING] master_joint=%.6f conservative_gap=%.6f "
             "required=%.6f pass=%s", measured_joint, measured_opening,
             required_opening,
             measured_opening > required_opening ? "true" : "false");
    return measured_opening > required_opening;
  }

  bool verifyPoseReached(const geometry_msgs::PoseStamped& target,
                         const char* state,
                         bool enforce_target_tolerance = true) {
    ros::Duration(0.10).sleep();
    const geometry_msgs::PoseStamped actual =
        arm_.getCurrentPose("gripper_tcp_link");
    if (actual.header.frame_id != target.header.frame_id) {
      ROS_ERROR("[POSE_AUDIT] state=%s frame mismatch actual=%s target=%s",
                state, actual.header.frame_id.c_str(),
                target.header.frame_id.c_str());
      return false;
    }
    const double position_error =
        posePositionError(actual.pose, target.pose);
    const double orientation_error =
        poseOrientationError(actual.pose, target.pose);
    const bool finite = std::isfinite(position_error) &&
        std::isfinite(orientation_error);
    const bool within_tolerance =
        position_error <= pose_position_tolerance_ &&
        orientation_error <= pose_orientation_tolerance_;
    const bool passed = finite &&
        (!enforce_target_tolerance || within_tolerance);
    ROS_INFO("[POSE_AUDIT] state=%s frame=%s actual=[%.5f,%.5f,%.5f] "
             "target=[%.5f,%.5f,%.5f] position_error=%.6f "
             "orientation_error=%.6f target_tolerance_enforced=%s pass=%s",
             state, target.header.frame_id.c_str(),
             actual.pose.position.x, actual.pose.position.y,
             actual.pose.position.z, target.pose.position.x,
             target.pose.position.y, target.pose.position.z,
             position_error, orientation_error,
             enforce_target_tolerance ? "true" : "false",
             passed ? "true" : "false");
    return passed;
  }

  bool cartesianMove(const geometry_msgs::PoseStamped& target,
                     const char* state,
                     bool enforce_target_tolerance = true) {
    ROS_INFO("[%s] computing vertical Cartesian path", state);
    const geometry_msgs::PoseStamped current =
        arm_.getCurrentPose("gripper_tcp_link");
    if (current.header.frame_id != target.header.frame_id) {
      ROS_ERROR("[%s] Cartesian frame mismatch actual=%s target=%s", state,
                current.header.frame_id.c_str(), target.header.frame_id.c_str());
      return false;
    }
    const double dx = target.pose.position.x - current.pose.position.x;
    const double dy = target.pose.position.y - current.pose.position.y;
    const double dz = target.pose.position.z - current.pose.position.z;
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    ROS_INFO("[POSE_AUDIT] state=%s current=[%.5f,%.5f,%.5f] "
             "target=[%.5f,%.5f,%.5f] distance=%.5f",
             state, current.pose.position.x, current.pose.position.y,
             current.pose.position.z, target.pose.position.x,
             target.pose.position.y, target.pose.position.z, distance);
    std::vector<geometry_msgs::Pose> waypoints(1, target.pose);
    moveit_msgs::RobotTrajectory trajectory;
    const double fraction = arm_.computeCartesianPath(waypoints, eef_step_, 0.0,
                                                       trajectory, true);
    if (fraction < min_fraction_) {
      ROS_ERROR("[%s] Cartesian fraction %.3f is below %.3f", state, fraction,
                min_fraction_);
      return false;
    }
    if (trajectory.joint_trajectory.points.empty()) {
      ROS_ERROR("[%s] Cartesian trajectory is empty", state);
      return false;
    }
    const double original_duration =
        trajectory.joint_trajectory.points.back().time_from_start.toSec();
    std::string timing_error;
    if (!retimeForCartesianSpeed(&trajectory.joint_trajectory, distance,
                                 approach_speed_, &timing_error)) {
      ROS_ERROR("[%s] Cartesian retiming failed: %s", state,
                timing_error.c_str());
      return false;
    }
    const double retimed_duration =
        trajectory.joint_trajectory.points.back().time_from_start.toSec();
    ROS_INFO("[CARTESIAN_TIMING] state=%s distance=%.4f speed=%.4f "
             "original_duration=%.4f retimed_duration=%.4f",
             state, distance, approach_speed_, original_duration,
             retimed_duration);
    if (arm_.execute(trajectory) !=
        moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      return false;
    }
    return verifyPoseReached(target, state, enforce_target_tolerance);
  }

  bool brickWorldHeight(double* height) {
    if (!height) return false;
    ros::ServiceClient client =
        nh_.serviceClient<gazebo_msgs::GetModelState>(
            "/gazebo/get_model_state");
    if (!client.waitForExistence(ros::Duration(5.0))) return false;
    gazebo_msgs::GetModelState request;
    request.request.model_name = brick_model_;
    request.request.relative_entity_name = "world";
    if (!client.call(request) || !request.response.success ||
        !std::isfinite(request.response.pose.position.z)) {
      return false;
    }
    *height = request.response.pose.position.z;
    return true;
  }

  bool setAttachment(bool attach) {
    if (!use_attacher_) return true;
    ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>(attachment_service_);
    if (!client.waitForExistence(ros::Duration(10.0))) return false;
    std_srvs::SetBool request;
    request.request.data = attach;
    if (!client.call(request)) {
      ROS_ERROR("[VERIFY_GRASP] attachment service transport failed");
      return false;
    }
    if (!request.response.success)
      ROS_ERROR("[VERIFY_GRASP] %s", request.response.message.c_str());
    else
      ROS_INFO("[VERIFY_GRASP] %s", request.response.message.c_str());
    return request.response.success;
  }

  void fail(const std::string& message) {
    ROS_ERROR("[ERROR] %s", message.c_str());
  }

  void poseCallback(const geometry_msgs::PoseStampedConstPtr& brick) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_) return;
    running_ = true;
    const GripperFeasibility feasibility = checkGripperOpening(
        dimensions_, parameters_.grasp_along_short_axis, maximum_opening_,
        opening_margin_);
    ROS_INFO("[GRIPPER_GEOMETRY] required=%.6f max=%.6f margin=%.6f "
             "feasible=%s", feasibility.required_opening, maximum_opening_,
             opening_margin_, feasibility.feasible ? "true" : "false");
    if (!feasibility.feasible) {
      return fail(feasibility.reason +
                  ": required opening exceeds actual AG95 jaw separation");
    }
    ROS_INFO("[OPEN_GRIPPER] commanding AG95 open");
    if (!commandGripper(open_position_, false, true)) {
      ROS_WARN("[OPEN_GRIPPER] first command failed; retrying once");
      if (!commandGripper(open_position_, false, true)) return fail("AG95 open failed");
    }
    if (!verifyPreOpen(feasibility.required_opening))
      return fail("AG95 measured opening does not exceed required opening");

    ROS_INFO("[GENERATE_GRASP] generating geometry-based top-down poses");
    GraspPoses poses;
    std::string error;
    if (!generateTopDownGrasp(*brick, dimensions_, parameters_, &poses, &error))
      return fail(error);
    ROS_INFO("[GRASP_POSES] grasp=[%.5f,%.5f,%.5f] "
             "pregrasp=[%.5f,%.5f,%.5f] lift=[%.5f,%.5f,%.5f] "
             "pad_lower_edge_offset=%.5f contact_overlap=%.5f",
             poses.grasp.pose.position.x, poses.grasp.pose.position.y,
             poses.grasp.pose.position.z,
             poses.pre_grasp.pose.position.x, poses.pre_grasp.pose.position.y,
             poses.pre_grasp.pose.position.z,
             poses.lift.pose.position.x, poses.lift.pose.position.y,
             poses.lift.pose.position.z,
             parameters_.finger_pad_lower_edge_offset,
             parameters_.finger_pad_contact_overlap);
    GraspPoses model_poses;
    const std::string model_frame = arm_.getPlanningFrame();
    if (!transformGraspPoses(poses, model_frame, &tf_buffer_,
                             ros::Duration(0.20), &model_poses, &error)) {
      return fail("grasp-pose TF failed: " + error);
    }
    ROS_INFO("[GRASP_TF] source=%s target=%s stamp=%u.%09u "
             "grasp=[%.5f,%.5f,%.5f]",
             poses.grasp.header.frame_id.c_str(), model_frame.c_str(),
             poses.grasp.header.stamp.sec, poses.grasp.header.stamp.nsec,
             model_poses.grasp.pose.position.x,
             model_poses.grasp.pose.position.y,
             model_poses.grasp.pose.position.z);

    ROS_INFO("[PLAN_PREGRASP] planning with RRTConnect");
    arm_.setStartStateToCurrentState();
    arm_.setPoseTarget(model_poses.pre_grasp, "gripper_tcp_link");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (arm_.plan(plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
      return fail("pre-grasp planning failed");
    if (!plan.trajectory_.joint_trajectory.points.empty()) {
      ROS_INFO("[PLAN_PREGRASP] points=%zu duration=%.5f",
               plan.trajectory_.joint_trajectory.points.size(),
               plan.trajectory_.joint_trajectory.points.back()
                   .time_from_start.toSec());
    }
    ROS_INFO("[MOVE_PREGRASP] executing planned trajectory");
    if (arm_.execute(plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
      return fail("pre-grasp execution failed");
    if (!verifyPoseReached(model_poses.pre_grasp, "MOVE_PREGRASP"))
      return fail("pre-grasp pose verification failed");
    arm_.stop();
    arm_.clearPoseTargets();

    if (!cartesianMove(model_poses.grasp, "CARTESIAN_APPROACH"))
      return fail("Cartesian approach failed");
    ROS_INFO("[CLOSE_GRIPPER] commanding AG95 close");
    if (!commandGripper(closed_position_, true)) return fail("AG95 close failed");
    ros::Duration(settle_time_).sleep();
    double initial_brick_height = 0.0;
    if (use_attacher_ && !brickWorldHeight(&initial_brick_height))
      return fail("initial Gazebo Brick height is unavailable");
    ROS_INFO("[VERIFY_GRASP] applying guarded simulation attachment");
    if (!setAttachment(true)) return fail("grasp attachment/verification failed");
    if (!cartesianMove(model_poses.lift, "LIFT", false))
      return fail("Cartesian lift failed");
    if (use_attacher_) {
      double final_brick_height = 0.0;
      if (!brickWorldHeight(&final_brick_height))
        return fail("final Gazebo Brick height is unavailable");
      double measured_lift = 0.0;
      const bool lift_passed = verifiedLift(
          initial_brick_height, final_brick_height, minimum_brick_lift_,
          &measured_lift);
      ROS_INFO("[VERIFY_LIFT] initial_z=%.6f final_z=%.6f delta=%.6f "
               "minimum=%.6f pass=%s", initial_brick_height,
               final_brick_height, measured_lift, minimum_brick_lift_,
               lift_passed ? "true" : "false");
      if (!lift_passed) return fail("Brick did not achieve minimum physical lift");
    }
    ROS_INFO("[SUCCESS] brick pick and lift completed");
    ros::param::set("/brick_pick_demo/success", true);
  }

  ros::NodeHandle nh_, pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Subscriber subscriber_;
  moveit::planning_interface::MoveGroupInterface arm_;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> gripper_;
  BrickDimensions dimensions_;
  GraspParameters parameters_;
  double eef_step_, min_fraction_, approach_speed_;
  double pose_position_tolerance_, pose_orientation_tolerance_;
  double open_position_, open_tolerance_;
  double closed_position_, closed_threshold_;
  double gripper_motion_time_, settle_time_, maximum_opening_;
  double maximum_joint_position_, opening_margin_;
  double minimum_brick_lift_;
  bool use_attacher_{true}, running_{false};
  std::string attachment_service_, brick_model_;
  std::mutex mutex_;
};

}  // namespace brick_pick_demo

int main(int argc, char** argv) {
  ros::init(argc, argv, "brick_pick_node");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  brick_pick_demo::BrickPickNode node;
  ros::waitForShutdown();
  return 0;
}
