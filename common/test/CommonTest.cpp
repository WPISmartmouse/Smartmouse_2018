#include "gtest/gtest.h"

#include <common/KinematicController/KinematicController.h>
#include <common/math/math.h>

TEST(ForwardKinematicsTest, Forward_one_second) {
  GlobalPose d_pose = KinematicController::forwardKinematics(1, 1, 0, 1);

  EXPECT_EQ(d_pose.col, 1);
  EXPECT_EQ(d_pose.row, 0);
  EXPECT_EQ(d_pose.yaw, 0);
}

TEST(ForwardKinematicsTest, Forward_tenth_second) {
  GlobalPose d_pose = KinematicController::forwardKinematics(1, 1, 0, 0.1);

  EXPECT_DOUBLE_EQ(d_pose.col, 0.1);
  EXPECT_DOUBLE_EQ(d_pose.row, 0);
  EXPECT_DOUBLE_EQ(d_pose.yaw, 0);
}

TEST(ForwardKinematicsTest, half_cw_turn_one_second) {
  GlobalPose d_pose = KinematicController::forwardKinematics(smartmouse::kc::TRACK_WIDTH_CU * M_PI, 0, 0, 1);

  EXPECT_NEAR(d_pose.col, 0, 1e-9);
  EXPECT_DOUBLE_EQ(d_pose.row, smartmouse::kc::TRACK_WIDTH_CU);
  EXPECT_DOUBLE_EQ(d_pose.yaw, M_PI);
}

TEST(ForwardKinematicsTest, half_ccw_turn_one_second) {
  GlobalPose d_pose = KinematicController::forwardKinematics(0, smartmouse::kc::TRACK_WIDTH_CU * M_PI, 0, 1);

  EXPECT_NEAR(d_pose.col, 0, 1e-9);
  EXPECT_DOUBLE_EQ(d_pose.row, -smartmouse::kc::TRACK_WIDTH_CU);
  EXPECT_DOUBLE_EQ(d_pose.yaw, -M_PI);
}

TEST(ForwardKinematicsTest, cw_turn_one_second) {
  GlobalPose d_pose = KinematicController::forwardKinematics(smartmouse::kc::TRACK_WIDTH_CU * M_PI, -smartmouse::kc::TRACK_WIDTH_CU * M_PI, 0, 1);

  EXPECT_NEAR(d_pose.col, 0, 1e-9);
  EXPECT_NEAR(d_pose.row, 0, 1e-9);
  EXPECT_DOUBLE_EQ(d_pose.yaw, 2 * M_PI);
}

TEST(ForwardKinematicsTest, ccw_turn_one_second) {
  GlobalPose d_pose = KinematicController::forwardKinematics(-smartmouse::kc::TRACK_WIDTH_CU * M_PI, smartmouse::kc::TRACK_WIDTH_CU * M_PI, 0, 1);

  EXPECT_NEAR(d_pose.col, 0, 1e-9);
  EXPECT_NEAR(d_pose.row, 0, 1e-9);
  EXPECT_DOUBLE_EQ(d_pose.yaw, -2 * M_PI);
}

TEST(YawDiffTest, taw_diff_1) {
  EXPECT_NEAR(smartmouse::math::yaw_diff(0, 0.1), 0.1, 1e-6);
}

TEST(YawDiffTest, taw_diff_2) {
  EXPECT_NEAR(smartmouse::math::yaw_diff(0, M_PI), M_PI, 1e-6);
}

TEST(YawDiffTest, taw_diff_3) {
  EXPECT_NEAR(smartmouse::math::yaw_diff(M_PI, 0), -M_PI, 1e-6);
}

TEST(YawDiffTest, taw_diff_4) {
  EXPECT_NEAR(smartmouse::math::yaw_diff(-M_PI_2, M_PI_2 + 0.1), -M_PI + 0.1, 1e-6);
}

TEST(YawDiffTest, taw_diff_5) {
  EXPECT_NEAR(smartmouse::math::yaw_diff(M_PI_2, -M_PI_2 - 0.1), M_PI - 0.1, 1e-6);
}

TEST(YawDiffTest, taw_diff_6) {
  EXPECT_NEAR(smartmouse::math::yaw_diff(dir_to_yaw(Direction::N), -M_PI_2 + 0.1), 0.1, 1e-6);
}

TEST(YawDiffTest, taw_diff_7) {
  EXPECT_NEAR(smartmouse::math::yaw_diff(M_PI - 0.1, -M_PI + 0.1), 0.2, 1e-6);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
