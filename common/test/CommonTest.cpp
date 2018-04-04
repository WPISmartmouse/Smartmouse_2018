#include "gtest/gtest.h"

#include <common/KinematicController/KinematicController.h>
#include <common/math/math.h>
#include <common/KinematicController/VelocityProfile.h>

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
  GlobalPose d_pose = KinematicController::forwardKinematics(smartmouse::kc::TRACK_WIDTH_CU * M_PI,
                                                             -smartmouse::kc::TRACK_WIDTH_CU * M_PI,
                                                             0,
                                                             1);

  EXPECT_NEAR(d_pose.col, 0, 1e-9);
  EXPECT_NEAR(d_pose.row, 0, 1e-9);
  EXPECT_DOUBLE_EQ(d_pose.yaw, 2 * M_PI);
}

TEST(ForwardKinematicsTest, ccw_turn_one_second) {
  GlobalPose d_pose = KinematicController::forwardKinematics(-smartmouse::kc::TRACK_WIDTH_CU * M_PI,
                                                             smartmouse::kc::TRACK_WIDTH_CU * M_PI,
                                                             0,
                                                             1);

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

TEST(FromSensorsToWallTest, from_sensors_to_wall_left_1) {
  // negative is counter clockwise--see dynamic_model pdf for justification
  smartmouse::kc::SensorPose back_left{-1, 0, -M_PI_2};
  smartmouse::kc::SensorPose front_left{1, 0, -M_PI_2};
  double d;
  std::tie(d, std::ignore) = smartmouse::kc::from_sensors_to_left_wall(back_left, front_left, 1, 1);
  EXPECT_NEAR(d, 1, 1e-6);
}

TEST(FromSensorsToWallTest, from_sensors_to_wall_left_2) {
  // TODO: negatives
  smartmouse::kc::SensorPose back_left{1, 0, -M_PI_2};
  smartmouse::kc::SensorPose front_left{2, -0.5, -M_PI_2};
  double d;
  std::tie(d, std::ignore) = smartmouse::kc::from_sensors_to_left_wall(back_left, front_left, 1, 0.5);
  EXPECT_NEAR(d, 1, 1e-6);
}

TEST(FromSensorsToWallTest, from_sensors_to_wall_left_3) {
  smartmouse::kc::SensorPose back_left{-sqrt(2), 0, -M_PI_4};
  smartmouse::kc::SensorPose front_left{sqrt(2), 0, -M_PI_4};
  double d;
  std::tie(d, std::ignore) = smartmouse::kc::from_sensors_to_left_wall(back_left, front_left, 3, 1);
  EXPECT_NEAR(d, 2, 1e-6);
}

TEST(FromSensorsToWallTest, from_sensors_to_wall_right_1) {
  smartmouse::kc::SensorPose back_right{-1, 0, M_PI_2};
  smartmouse::kc::SensorPose front_right{1, 0, M_PI_2};
  double d;
  std::tie(d, std::ignore) = smartmouse::kc::from_sensors_to_right_wall(back_right, front_right, 1, 1);
  EXPECT_NEAR(d, 1, 1e-6);
}

TEST(FromSensorsToWallTest, from_sensors_to_wall_right_2) {
  // TODO: negatives
  smartmouse::kc::SensorPose back_right{1, 0, M_PI_2};
  smartmouse::kc::SensorPose front_right{2, 0.5, M_PI_2};
  double d;
  std::tie(d, std::ignore) = smartmouse::kc::from_sensors_to_right_wall(back_right, front_right, 1, 0.5);
  EXPECT_NEAR(d, 1, 1e-6);
}

TEST(FromSensorsToWallTest, from_sensors_to_wall_right_3) {
  smartmouse::kc::SensorPose back_right{-sqrt(2), 0, M_PI_4};
  smartmouse::kc::SensorPose front_right{sqrt(2), 0, M_PI_4};
  double d;
  std::tie(d, std::ignore) = smartmouse::kc::from_sensors_to_right_wall(back_right, front_right, 3, 1);
  EXPECT_NEAR(d, 2, 1e-6);
}

TEST(VelocityProfileTest, ForwardTest5) {
  GlobalPose start(0, 0, 0);
  smartmouse::kc::VelocityProfileTiming timing(5, 0, 0);
  auto profile = smartmouse::kc::VelocityProfile(start, timing);

  auto v0 = profile.compute_forward_velocity(0);
  auto vf = profile.compute_forward_velocity(timing.t_f+1e-6);
  auto v_middle = profile.compute_forward_velocity((timing.t_m1 + timing.t_m2) / 2);
  EXPECT_EQ(v0, 0);
  EXPECT_EQ(v_middle, smartmouse::kc::MAX_SPEED_CUPS);
  EXPECT_EQ(vf, 0);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
