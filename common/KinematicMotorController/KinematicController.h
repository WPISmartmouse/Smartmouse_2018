#pragma once

#include <utility>
#include <common/Pose.h>
#include "RegulatedMotor.h"

class KinematicMotorController {
public:
  KinematicMotorController();

  Pose getPose();

  std::pair<double, double> getWheelVelocities();

  bool isStopped();

  void reset_x_to(double new_x);

  void reset_y_to(double new_y);

  void reset_yaw_to(double new_yaw);

  std::pair<double, double> run(unsigned long time_ms, double left_angle_rad, double right_angle_rad);

  void setAcceleration(double acceleration, double break_acceleration);

  void setSpeedMps(double left_setpoint_mps, double right_setpoint_mps);

private:
  bool initialized = false;
  unsigned long last_run_time_ms;

  Pose current_pose_estimate;
  RegulatedMotor left_motor;
  RegulatedMotor right_motor;

};