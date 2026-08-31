#include <brick_pick_demo/pose_frames.h>

#include <gtest/gtest.h>
#include <geometry_msgs/TransformStamped.h>

#include <limits>

namespace {
brick_pick_demo::GraspPoses posesInBase() {
  brick_pick_demo::GraspPoses poses;
  poses.grasp.header.frame_id = "aubo_i5_base_link";
  poses.grasp.header.stamp = ros::Time(12, 340000000);
  poses.grasp.pose.position.x = 0.60;
  poses.grasp.pose.position.z = -0.4565;
  poses.grasp.pose.orientation.w = 1.0;
  poses.pre_grasp = poses.grasp;
  poses.pre_grasp.pose.position.z += 0.15;
  poses.lift = poses.grasp;
  poses.lift.pose.position.z += 0.15;
  return poses;
}
}

TEST(PoseFrames, UsesExactStampedTransformForEveryCartesianPose) {
  tf2_ros::Buffer buffer;
  buffer.setUsingDedicatedThread(true);
  geometry_msgs::TransformStamped transform;
  transform.header.frame_id = "world";
  transform.child_frame_id = "aubo_i5_base_link";
  transform.header.stamp = ros::Time(12, 340000000);
  transform.transform.translation.x = 0.15;
  transform.transform.translation.z = 0.483;
  transform.transform.rotation.w = 1.0;
  ASSERT_TRUE(buffer.setTransform(transform, "test", false));

  brick_pick_demo::GraspPoses result;
  std::string error;
  ASSERT_TRUE(brick_pick_demo::transformGraspPoses(
      posesInBase(), "world", &buffer, ros::Duration(0.0), &result, &error))
      << error;
  EXPECT_EQ("world", result.grasp.header.frame_id);
  EXPECT_EQ(posesInBase().grasp.header.stamp, result.grasp.header.stamp);
  EXPECT_NEAR(0.75, result.grasp.pose.position.x, 1e-12);
  EXPECT_NEAR(0.0265, result.grasp.pose.position.z, 1e-12);
  EXPECT_NEAR(0.1765, result.pre_grasp.pose.position.z, 1e-12);
  EXPECT_NEAR(0.1765, result.lift.pose.position.z, 1e-12);
}

TEST(PoseFrames, MissingMeasurementTimeTransformFailsClosed) {
  tf2_ros::Buffer buffer;
  buffer.setUsingDedicatedThread(true);
  brick_pick_demo::GraspPoses result;
  std::string error;
  EXPECT_FALSE(brick_pick_demo::transformGraspPoses(
      posesInBase(), "world", &buffer, ros::Duration(0.0), &result, &error));
  EXPECT_FALSE(error.empty());
}

TEST(PoseFrames, ComputesCartesianPositionError) {
  geometry_msgs::Pose actual;
  geometry_msgs::Pose target;
  target.position.x = 0.003;
  target.position.y = 0.004;
  EXPECT_NEAR(0.005, brick_pick_demo::posePositionError(actual, target),
              1e-12);
}

TEST(PoseFrames, ComputesShortestQuaternionOrientationError) {
  geometry_msgs::Pose actual;
  geometry_msgs::Pose target;
  actual.orientation.w = 1.0;
  target.orientation.z = std::sin(0.05);
  target.orientation.w = std::cos(0.05);
  EXPECT_NEAR(0.10, brick_pick_demo::poseOrientationError(actual, target),
              1e-12);
  target.orientation.z *= -1.0;
  target.orientation.w *= -1.0;
  EXPECT_NEAR(0.10, brick_pick_demo::poseOrientationError(actual, target),
              1e-12);
}

TEST(PoseFrames, VerifiesActualBrickLiftAndRejectsInvalidEvidence) {
  double lift = 0.0;
  EXPECT_TRUE(brick_pick_demo::verifiedLift(0.0575, 0.1780, 0.10, &lift));
  EXPECT_NEAR(0.1205, lift, 1e-12);
  EXPECT_FALSE(brick_pick_demo::verifiedLift(0.0575, 0.1500, 0.10, &lift));
  EXPECT_FALSE(brick_pick_demo::verifiedLift(
      std::numeric_limits<double>::quiet_NaN(), 0.20, 0.10, &lift));
  EXPECT_FALSE(brick_pick_demo::verifiedLift(0.05, 0.20, 0.0, &lift));
  EXPECT_FALSE(brick_pick_demo::verifiedLift(0.05, 0.20, 0.10, nullptr));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();
  return RUN_ALL_TESTS();
}
