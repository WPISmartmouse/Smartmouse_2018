#pragma once

#include <utility>
#include <common/Pose.h>
#include "RegulatedMotor.h"

class KinematicMotorController {
public:
  KinematicMotorController();

  Pose get_pose();

  void reset_x_to(double new_x);

  void reset_y_to(double new_y);

  void reset_yaw_to(double new_yaw);

  std::pair<double, double> run(unsigned long time_ms, double left_angle_rad, double right_angle_rad);

  void setAcceleration(double acceleration, double break_acceleration);

  void setSpeed(double left_wheel_velocity_setpoint_mps, double right_wheel_velocity_setpoint_mps);

private:
  bool initialized = false;
  double acceleration;
  double brake_acceleration;
  double left_setpoint_mps;
  double right_setpoint_mps;
  unsigned long last_run_time_ms;
  unsigned  long last_control_run_time_ms;

  Pose current_pose_estimate;
  RegulatedMotor left_motor;
  RegulatedMotor right_motor;

};