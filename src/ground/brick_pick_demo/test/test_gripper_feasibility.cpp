#include <brick_pick_demo/brick_geometry.h>
#include <brick_pick_demo/gripper_feasibility.h>

#include <gtest/gtest.h>

namespace brick_pick_demo {

TEST(BrickGeometry, DerivesFlatAndSideUpWithoutChangingNominalDimensions) {
  BrickDimensions brick;
  brick.length = 0.240;
  brick.width = 0.115;
  brick.height = 0.053;
  OrientedBrickGeometry oriented;
  std::string error;

  brick.orientation_mode = "flat";
  ASSERT_TRUE(deriveOrientedBrickGeometry(brick, &oriented, &error)) << error;
  EXPECT_NEAR(0.240, oriented.top_length, 1e-12);
  EXPECT_NEAR(0.115, oriented.top_width, 1e-12);
  EXPECT_NEAR(0.053, oriented.vertical_height, 1e-12);
  EXPECT_NEAR(0.115, oriented.grasp_span, 1e-12);
  EXPECT_NEAR(0.0265, oriented.resting_center_z, 1e-12);

  brick.orientation_mode = "side_up";
  ASSERT_TRUE(deriveOrientedBrickGeometry(brick, &oriented, &error)) << error;
  EXPECT_NEAR(0.240, oriented.top_length, 1e-12);
  EXPECT_NEAR(0.053, oriented.top_width, 1e-12);
  EXPECT_NEAR(0.115, oriented.vertical_height, 1e-12);
  EXPECT_NEAR(0.053, oriented.grasp_span, 1e-12);
  EXPECT_NEAR(0.0575, oriented.resting_center_z, 1e-12);
  EXPECT_NEAR(0.115, brick.width, 1e-12);
  EXPECT_NEAR(0.053, brick.height, 1e-12);
}

TEST(BrickGeometry, RejectsInvalidModeAndDimensions) {
  BrickDimensions brick;
  OrientedBrickGeometry oriented;
  std::string error;
  brick.orientation_mode = "edge";
  EXPECT_FALSE(deriveOrientedBrickGeometry(brick, &oriented, &error));
  EXPECT_EQ("INVALID_BRICK_ORIENTATION_MODE", error);
  brick.orientation_mode = "side_up";
  brick.height = 0.0;
  EXPECT_FALSE(deriveOrientedBrickGeometry(brick, &oriented, &error));
  EXPECT_EQ("INVALID_BRICK_DIMENSIONS", error);
}

TEST(GripperFeasibility, RejectsCurrentBrickShortAxis) {
  BrickDimensions brick;
  brick.length = 0.240;
  brick.width = 0.115;
  brick.height = 0.053;
  brick.orientation_mode = "flat";
  const GripperFeasibility result =
      checkGripperOpening(brick, true, 0.0952, 0.002);
  EXPECT_FALSE(result.feasible);
  EXPECT_NEAR(0.115, result.grasp_span, 1e-12);
  EXPECT_NEAR(0.117, result.required_opening, 1e-12);
  EXPECT_EQ("PHYSICALLY_INFEASIBLE_WITH_CURRENT_GRASP", result.reason);
}

TEST(GripperFeasibility, AcceptsSideUpThicknessWithClearance) {
  BrickDimensions brick;
  brick.length = 0.240;
  brick.width = 0.115;
  brick.height = 0.053;
  brick.orientation_mode = "side_up";
  const GripperFeasibility result =
      checkGripperOpening(brick, true, 0.0952, 0.002);
  EXPECT_TRUE(result.feasible);
  EXPECT_NEAR(0.053, result.grasp_span, 1e-12);
  EXPECT_NEAR(0.055, result.required_opening, 1e-12);
  EXPECT_TRUE(result.reason.empty());
}

TEST(GripperFeasibility, AcceptsTargetWithinRealOpeningAndMargin) {
  BrickDimensions brick;
  brick.length = 0.080;
  brick.width = 0.060;
  brick.height = 0.040;
  const GripperFeasibility result =
      checkGripperOpening(brick, true, 0.0952, 0.002);
  EXPECT_TRUE(result.feasible);
  EXPECT_NEAR(0.060, result.grasp_span, 1e-12);
  EXPECT_NEAR(0.062, result.required_opening, 1e-12);
  EXPECT_TRUE(result.reason.empty());
}

TEST(GripperFeasibility, UsesLongAxisWhenRequested) {
  BrickDimensions brick;
  brick.length = 0.120;
  brick.width = 0.060;
  brick.height = 0.040;
  const GripperFeasibility result =
      checkGripperOpening(brick, false, 0.0952, 0.0);
  EXPECT_FALSE(result.feasible);
  EXPECT_NEAR(0.120, result.grasp_span, 1e-12);
  EXPECT_NEAR(0.120, result.required_opening, 1e-12);
}

TEST(GripperFeasibility, VerifiesMeasuredOpeningConservatively) {
  double opening = 0.0;
  std::string error;
  ASSERT_TRUE(estimateJawOpening(0.0, 0.0952, 0.93, &opening, &error))
      << error;
  EXPECT_NEAR(0.0952, opening, 1e-12);
  ASSERT_TRUE(estimateJawOpening(0.05, 0.0952, 0.93, &opening, &error))
      << error;
  EXPECT_NEAR(0.0952 * (1.0 - 0.05 / 0.93), opening, 1e-12);
  EXPECT_GT(opening, 0.055);
  EXPECT_FALSE(estimateJawOpening(-0.01, 0.0952, 0.93, &opening, &error));
  EXPECT_EQ("INVALID_GRIPPER_JOINT_MEASUREMENT", error);
  EXPECT_FALSE(estimateJawOpening(0.0, 0.0952, 0.0, &opening, &error));
  EXPECT_EQ("INVALID_GRIPPER_CALIBRATION", error);
}

}  // namespace brick_pick_demo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
