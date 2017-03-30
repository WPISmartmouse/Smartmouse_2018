#pragma once

#include <utility>
#include <common/Pose.h>
#include "RegulatedMotor.h"

class KinematicMotorController {
public:
  KinematicMotorController(unsigned long period_ms);

  Pose get_pose();

  std::pair<double, double> run(unsigned long time_ms, double left_angle_rad, double right_angle_rad);

  void setAcceleration(double start_acceleration, double stop_acceleration);

  void setSpeed(double left_wheel_velocity_setpoint_mps, double right_wheel_velocity_setpoint_mps);

private:
  double start_acceleration;
  double stop_acceleration;

  Pose current_pose_estimate;
  RegulatedMotor left_motor;
  RegulatedMotor right_motor;

};