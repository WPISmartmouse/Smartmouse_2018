#include "gtest/gtest.h"

#include <common/KinematicController/KinematicController.h>

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

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
