#include <bunker_aubo_gazebo/ag95_effort_coupling.h>

#include <gtest/gtest.h>

#include <limits>

namespace bunker_aubo_gazebo {

TEST(Ag95EffortCoupling, OpposesFinitePositionAndVelocityError) {
  EffortCouplingParameters parameters;
  parameters.position_gain = 20.0;
  parameters.velocity_gain = 1.5;
  parameters.maximum_effort = 50.0;
  EffortCouplingResult result = calculateEffortCoupling(
      0.4, 0.2, 0.1, -0.1, parameters);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_NEAR(6.45, result.effort, 1e-12);
  result = calculateEffortCoupling(0.1, -0.1, 0.4, 0.2, parameters);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_LT(result.effort, 0.0);
}

TEST(Ag95EffortCoupling, ClampsWithoutChangingHardwareLimit) {
  EffortCouplingParameters parameters;
  parameters.position_gain = 1000.0;
  parameters.velocity_gain = 0.0;
  parameters.maximum_effort = 50.0;
  const EffortCouplingResult positive = calculateEffortCoupling(
      0.93, 0.0, 0.0, 0.0, parameters);
  const EffortCouplingResult negative = calculateEffortCoupling(
      0.0, 0.0, 0.93, 0.0, parameters);
  ASSERT_TRUE(positive.valid);
  ASSERT_TRUE(negative.valid);
  EXPECT_DOUBLE_EQ(50.0, positive.effort);
  EXPECT_DOUBLE_EQ(-50.0, negative.effort);
}

TEST(Ag95EffortCoupling, RejectsNonFiniteStateOrInvalidCalibration) {
  EffortCouplingParameters parameters;
  parameters.position_gain = 20.0;
  parameters.velocity_gain = 1.5;
  parameters.maximum_effort = 50.0;
  EXPECT_FALSE(calculateEffortCoupling(
      std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0,
      parameters).valid);
  parameters.maximum_effort = 50.1;
  EXPECT_FALSE(calculateEffortCoupling(
      0.0, 0.0, 0.0, 0.0, parameters).valid);
}

}  // namespace bunker_aubo_gazebo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
