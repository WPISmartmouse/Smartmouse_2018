#include "gtest/gtest.h"

#include <common/KinematicController/KinematicController.h>

TEST(ForwardKinematicsTest, Forward_one_second) {
  double drow, dcol, dtheta;
  std::tie(dcol, drow, dtheta) = KinematicController::forwardKinematics(1, 1, 0, 1);

  EXPECT_EQ(dcol, 1);
  EXPECT_EQ(drow, 0);
  EXPECT_EQ(dtheta, 0);
}

TEST(ForwardKinematicsTest, Forward_tenth_second) {
  double drow, dcol, dtheta;
  std::tie(dcol, drow, dtheta) = KinematicController::forwardKinematics(1, 1, 0, 0.1);

  EXPECT_DOUBLE_EQ(dcol, 0.1);
  EXPECT_DOUBLE_EQ(drow, 0);
  EXPECT_DOUBLE_EQ(dtheta, 0);
}

TEST(ForwardKinematicsTest, half_cw_turn_one_second) {
  double drow, dcol, dtheta;
  std::tie(dcol, drow, dtheta) = KinematicController::forwardKinematics(smartmouse::kc::TRACK_WIDTH * M_PI, 0, 0, 1);

  EXPECT_NEAR(dcol, 0, 1e-9);
  EXPECT_DOUBLE_EQ(drow, smartmouse::kc::TRACK_WIDTH);
  EXPECT_DOUBLE_EQ(dtheta, M_PI);
}

TEST(ForwardKinematicsTest, half_ccw_turn_one_second) {
  double drow, dcol, dtheta;
  std::tie(dcol, drow, dtheta) = KinematicController::forwardKinematics(0, smartmouse::kc::TRACK_WIDTH * M_PI, 0, 1);

  EXPECT_NEAR(dcol, 0, 1e-9);
  EXPECT_DOUBLE_EQ(drow, -smartmouse::kc::TRACK_WIDTH);
  EXPECT_DOUBLE_EQ(dtheta, -M_PI);
}

TEST(ForwardKinematicsTest, cw_turn_one_second) {
  double drow, dcol, dtheta;
  std::tie(dcol, drow, dtheta) = KinematicController::forwardKinematics(smartmouse::kc::TRACK_WIDTH * M_PI, -smartmouse::kc::TRACK_WIDTH * M_PI, 0, 1);

  EXPECT_NEAR(dcol, 0, 1e-9);
  EXPECT_NEAR(drow, 0, 1e-9);
  EXPECT_DOUBLE_EQ(dtheta, 2 * M_PI);
}

TEST(ForwardKinematicsTest, ccw_turn_one_second) {
  double drow, dcol, dtheta;
  std::tie(dcol, drow, dtheta) = KinematicController::forwardKinematics(-smartmouse::kc::TRACK_WIDTH * M_PI, smartmouse::kc::TRACK_WIDTH * M_PI, 0, 1);

  EXPECT_NEAR(dcol, 0, 1e-9);
  EXPECT_NEAR(drow, 0, 1e-9);
  EXPECT_DOUBLE_EQ(dtheta, -2 * M_PI);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
