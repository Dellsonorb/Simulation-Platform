#include <gtest/gtest.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <brick_pick_demo/grasp_geometry.h>

namespace {
geometry_msgs::PoseStamped brick(double yaw = 0.4) {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "world";
  pose.pose.position.x = 0.7;
  pose.pose.position.y = 0.1;
  pose.pose.position.z = 0.0265;
  tf2::Quaternion q; q.setRPY(0, 0, yaw);
  pose.pose.orientation = tf2::toMsg(q);
  return pose;
}
}

TEST(GraspGeometry, SideUpGeneratesPadCenteredVerticalSequence) {
  brick_pick_demo::BrickDimensions dimensions;
  dimensions.orientation_mode = "side_up";
  brick_pick_demo::GraspParameters parameters;
  brick_pick_demo::GraspPoses poses;
  std::string error;
  geometry_msgs::PoseStamped side = brick();
  side.pose.position.z = 0.0575;
  ASSERT_TRUE(brick_pick_demo::generateTopDownGrasp(
      side, dimensions, parameters, &poses, &error)) << error;
  EXPECT_DOUBLE_EQ(0.7, poses.grasp.pose.position.x);
  // Pad lower edge is 15.6 mm behind TCP and is inserted 20 mm down the
  // side-up Brick, leaving the TCP 79.4 mm above the support plane.
  EXPECT_NEAR(0.0794, poses.grasp.pose.position.z, 1e-9);
  EXPECT_NEAR(0.2294, poses.pre_grasp.pose.position.z, 1e-9);
  EXPECT_NEAR(0.2294, poses.lift.pose.position.z, 1e-9);
  tf2::Quaternion q; tf2::fromMsg(poses.grasp.pose.orientation, q);
  tf2::Vector3 approach = tf2::quatRotate(q, tf2::Vector3(1, 0, 0));
  EXPECT_NEAR(0.0, approach.x(), 1e-6);
  EXPECT_NEAR(0.0, approach.y(), 1e-6);
  EXPECT_NEAR(-1.0, approach.z(), 1e-6);
}

TEST(GraspGeometry, SideUpUsesActualVerticalHeightAndKeepsYawDefinition) {
  brick_pick_demo::BrickDimensions dimensions;
  dimensions.orientation_mode = "side_up";
  brick_pick_demo::GraspParameters parameters;
  brick_pick_demo::GraspPoses poses;
  std::string error;
  geometry_msgs::PoseStamped side = brick(-0.35);
  side.pose.position.z = 0.0575;
  ASSERT_TRUE(brick_pick_demo::generateTopDownGrasp(
      side, dimensions, parameters, &poses, &error)) << error;
  EXPECT_NEAR(0.0794, poses.grasp.pose.position.z, 1e-9);
  EXPECT_NEAR(0.2294, poses.pre_grasp.pose.position.z, 1e-9);
  EXPECT_NEAR(0.2294, poses.lift.pose.position.z, 1e-9);
  tf2::Quaternion q;
  tf2::fromMsg(poses.grasp.pose.orientation, q);
  tf2::Vector3 finger_closing_axis =
      tf2::quatRotate(q, tf2::Vector3(0, 1, 0));
  EXPECT_NEAR(std::sin(0.35), finger_closing_axis.x(), 1e-6);
  EXPECT_NEAR(std::cos(0.35), finger_closing_axis.y(), 1e-6);
  EXPECT_NEAR(0.0, finger_closing_axis.z(), 1e-6);
}

TEST(GraspGeometry, SelectsAlternateBrickAxis) {
  brick_pick_demo::BrickDimensions dimensions;
  dimensions.orientation_mode = "side_up";
  brick_pick_demo::GraspParameters short_axis, long_axis;
  long_axis.grasp_along_short_axis = false;
  brick_pick_demo::GraspPoses a, b;
  std::string error;
  geometry_msgs::PoseStamped side = brick();
  side.pose.position.z = 0.0575;
  ASSERT_TRUE(brick_pick_demo::generateTopDownGrasp(side, dimensions, short_axis, &a, &error));
  ASSERT_TRUE(brick_pick_demo::generateTopDownGrasp(side, dimensions, long_axis, &b, &error));
  tf2::Quaternion qa, qb; tf2::fromMsg(a.grasp.pose.orientation, qa); tf2::fromMsg(b.grasp.pose.orientation, qb);
  tf2::Vector3 ya = tf2::quatRotate(qa, tf2::Vector3(0, 1, 0));
  tf2::Vector3 yb = tf2::quatRotate(qb, tf2::Vector3(0, 1, 0));
  EXPECT_NEAR(0.0, ya.dot(yb), 1e-6);
}

TEST(GraspGeometry, RejectsInvalidInput) {
  brick_pick_demo::BrickDimensions dimensions;
  brick_pick_demo::GraspParameters parameters;
  brick_pick_demo::GraspPoses poses;
  std::string error;
  auto pose = brick(); pose.header.frame_id = "camera_link";
  EXPECT_FALSE(brick_pick_demo::generateTopDownGrasp(pose, dimensions, parameters, &poses, &error));
  dimensions.width = 0.0;
  pose.header.frame_id = "world";
  EXPECT_FALSE(brick_pick_demo::generateTopDownGrasp(pose, dimensions, parameters, &poses, &error));
  dimensions.width = 0.115;
  parameters.surface_clearance = -0.001;
  EXPECT_FALSE(brick_pick_demo::generateTopDownGrasp(
      pose, dimensions, parameters, &poses, &error));
}

TEST(GraspGeometry, RejectsTcpInsertionBelowSupportingGroundPlane) {
  brick_pick_demo::BrickDimensions dimensions;
  brick_pick_demo::GraspParameters parameters;
  brick_pick_demo::GraspPoses poses;
  std::string error;
  parameters.finger_pad_contact_overlap = 0.050;
  EXPECT_FALSE(brick_pick_demo::generateTopDownGrasp(
      brick(), dimensions, parameters, &poses, &error));
  EXPECT_NE(std::string::npos, error.find("ground clearance"));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
