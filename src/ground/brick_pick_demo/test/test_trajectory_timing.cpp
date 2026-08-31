#include <brick_pick_demo/trajectory_timing.h>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

namespace {

trajectory_msgs::JointTrajectory trajectory(double duration = 1.0) {
  trajectory_msgs::JointTrajectory result;
  result.joint_names.push_back("joint");
  for (int index = 0; index < 3; ++index) {
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.push_back(0.5 * index);
    point.velocities.push_back(2.0);
    point.accelerations.push_back(4.0);
    point.time_from_start = ros::Duration(duration * index / 2.0);
    result.points.push_back(point);
  }
  return result;
}

}  // namespace

TEST(TrajectoryTiming, ExtendsFastCartesianTrajectoryAndScalesDerivatives) {
  auto value = trajectory();
  std::string error;
  ASSERT_TRUE(brick_pick_demo::retimeForCartesianSpeed(
      &value, 0.15, 0.03, &error)) << error;
  EXPECT_NEAR(5.0, value.points.back().time_from_start.toSec(), 1e-9);
  EXPECT_NEAR(0.4, value.points[1].velocities[0], 1e-9);
  EXPECT_NEAR(0.16, value.points[1].accelerations[0], 1e-9);
  ASSERT_EQ(1u, value.points.back().velocities.size());
  ASSERT_EQ(1u, value.points.back().accelerations.size());
  EXPECT_DOUBLE_EQ(0.0, value.points.back().velocities[0]);
  EXPECT_DOUBLE_EQ(0.0, value.points.back().accelerations[0]);
}

TEST(TrajectoryTiming, KeepsAnAlreadySlowDurationButStopsAtTheEndpoint) {
  auto value = trajectory(6.0);
  std::string error;
  ASSERT_TRUE(brick_pick_demo::retimeForCartesianSpeed(
      &value, 0.15, 0.03, &error)) << error;
  EXPECT_NEAR(6.0, value.points.back().time_from_start.toSec(), 1e-9);
  EXPECT_DOUBLE_EQ(2.0, value.points[1].velocities[0]);
  EXPECT_DOUBLE_EQ(4.0, value.points[1].accelerations[0]);
  EXPECT_DOUBLE_EQ(0.0, value.points.back().velocities[0]);
  EXPECT_DOUBLE_EQ(0.0, value.points.back().accelerations[0]);
}

TEST(TrajectoryTiming, RejectsMalformedAndNonFiniteInputs) {
  std::string error;
  auto decreasing = trajectory();
  decreasing.points[1].time_from_start = ros::Duration(2.0);
  EXPECT_FALSE(brick_pick_demo::retimeForCartesianSpeed(
      &decreasing, 0.15, 0.03, &error));
  EXPECT_FALSE(error.empty());

  auto non_finite = trajectory();
  non_finite.points[1].positions[0] =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(brick_pick_demo::retimeForCartesianSpeed(
      &non_finite, 0.15, 0.03, &error));

  auto valid = trajectory();
  EXPECT_FALSE(brick_pick_demo::retimeForCartesianSpeed(
      &valid, 0.15, 0.0, &error));
  EXPECT_FALSE(brick_pick_demo::retimeForCartesianSpeed(
      nullptr, 0.15, 0.03, &error));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
