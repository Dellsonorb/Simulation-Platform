#include <bunker_aubo_gazebo/attachment_guard.h>

#include <gtest/gtest.h>

namespace bunker_aubo_gazebo {

TEST(AttachmentGuard, RejectsInfeasibleTargetBeforeContactShortcut) {
  AttachmentEvidence evidence;
  evidence.preopened = true;
  evidence.gripper_closed = true;
  evidence.left_contact = true;
  evidence.right_contact = true;
  const AttachmentDecision result = decideAttachment(evidence, 0.003);
  EXPECT_FALSE(result.allowed);
  EXPECT_EQ("PHYSICALLY_INFEASIBLE_WITH_CURRENT_GRASP", result.reason);
}

TEST(AttachmentGuard, RequiresPreopenAndBilateralContacts) {
  AttachmentEvidence evidence;
  evidence.geometry_feasible = true;
  evidence.gripper_closed = true;
  evidence.left_contact = true;
  evidence.left_contact_count = 2;
  EXPECT_FALSE(decideAttachment(evidence, 0.003).allowed);
  evidence.preopened = true;
  EXPECT_FALSE(decideAttachment(evidence, 0.003).allowed);
  evidence.right_contact = true;
  evidence.right_contact_count = 2;
  EXPECT_FALSE(decideAttachment(evidence, 0.003).allowed);
  evidence.brick_bracketed = true;
  EXPECT_FALSE(decideAttachment(evidence, 0.003).allowed);
  evidence.opposed_contact_normals = true;
  EXPECT_TRUE(decideAttachment(evidence, 0.003).allowed);
}

TEST(AttachmentGuard, RejectsBooleanContactWithoutCountsOrEnclosure) {
  AttachmentEvidence evidence;
  evidence.geometry_feasible = true;
  evidence.preopened = true;
  evidence.gripper_closed = true;
  evidence.left_contact = true;
  evidence.right_contact = true;
  evidence.brick_bracketed = true;
  evidence.opposed_contact_normals = true;
  EXPECT_FALSE(decideAttachment(evidence, 0.003).allowed);
  evidence.left_contact_count = 1;
  evidence.right_contact_count = 1;
  evidence.brick_bracketed = false;
  EXPECT_FALSE(decideAttachment(evidence, 0.003).allowed);
  evidence.brick_bracketed = true;
  evidence.opposed_contact_normals = false;
  EXPECT_FALSE(decideAttachment(evidence, 0.003).allowed);
}

TEST(AttachmentGuard, RejectsIllegalPenetration) {
  AttachmentEvidence evidence;
  evidence.geometry_feasible = true;
  evidence.preopened = true;
  evidence.gripper_closed = true;
  evidence.left_contact = true;
  evidence.right_contact = true;
  evidence.left_contact_count = 1;
  evidence.right_contact_count = 1;
  evidence.brick_bracketed = true;
  evidence.opposed_contact_normals = true;
  evidence.maximum_penetration = 0.004;
  EXPECT_FALSE(decideAttachment(evidence, 0.003).allowed);
}

}  // namespace bunker_aubo_gazebo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
