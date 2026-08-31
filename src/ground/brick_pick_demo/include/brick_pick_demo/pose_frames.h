#pragma once

#include <brick_pick_demo/grasp_geometry.h>
#include <geometry_msgs/Pose.h>
#include <ros/duration.h>
#include <tf2_ros/buffer.h>

#include <string>

namespace brick_pick_demo {

bool transformGraspPoses(const GraspPoses& source,
                         const std::string& target_frame,
                         tf2_ros::Buffer* buffer,
                         const ros::Duration& timeout,
                         GraspPoses* result,
                         std::string* error);

double posePositionError(const geometry_msgs::Pose& actual,
                         const geometry_msgs::Pose& target);

double poseOrientationError(const geometry_msgs::Pose& actual,
                            const geometry_msgs::Pose& target);

bool verifiedLift(double initial_height, double final_height,
                  double minimum_lift, double* measured_lift);

}  // namespace brick_pick_demo
