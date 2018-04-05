#pragma once

#include <common/KinematicController/RobotConfig.h>

class RegulatedMotor {
public:
  RegulatedMotor();

  bool isStopped();

  double runPid(double dt_s, double angle_rad);

  void setAccelerationCpss(double acceleration_cupss);

  void setSetpointCps(double setpoint_cups);

  void setParams(double kP, double kI, double kD, double ff_scale, double ff_offset);

  double kP;
  double kI;
  double kD;
  double ff_offset;
  double ff_scale;
  double int_cap;

  bool initialized = false;
  double abstract_force;
  double acceleration_rpss;
  double derivative;
  double error;
  double feed_forward;
  double integral;
  double last_angle_rad;
  double last_error;
  double last_velocity_rps;
  double setpoint_rps;
  double velocity_rps;
  double second_derivative;
  double last_derivative;
};