#pragma once

#include <utility>
#include <common/util.h>
#include <common/Pose.h>
#include <common/RobotConfig.h>
#include <common/Mouse.h>
#include "RegulatedMotor.h"
#include <tuple>

class KinematicController {
public:
  KinematicController(Mouse *mouse);

  static double yawDiff(double y1, double y2);

  std::tuple<double, double, bool> estimate_pose(RangeData range_data, Mouse *mouse);

  GlobalPose getGlobalPose();

  LocalPose getLocalPose();

  std::pair<double, double> getWheelVelocities();

  bool isStopped();

  void reset_x_to(double new_x);

  void reset_y_to(double new_y);

  void reset_yaw_to(double new_yaw);

  std::pair<double, double>
  run(double dt_s, double left_angle_rad, double right_angle_rad, double ground_truth_left_vel_rps,
      double ground_truth_right_vel_rps, RangeData range_data);

  void setAcceleration(double acceleration, double break_acceleration);

  void setSpeedMps(double left_setpoint_mps, double right_setpoint_mps);

  RegulatedMotor left_motor;
  RegulatedMotor right_motor;

  bool ignore_sensor_pose_estimate;
  bool enabled;
  double row_offset_to_edge;
  double col_offset_to_edge;
  unsigned int row;
  unsigned int col;

private:
  bool initialized;
  bool ignoring_left;
  bool ignoring_right;

  GlobalPose current_pose_estimate;
  Mouse *mouse;
  double d_until_left_drop;
  double d_until_right_drop;
  static const double DROP_SAFETY;
  static const double POST_DROP_DIST;
};