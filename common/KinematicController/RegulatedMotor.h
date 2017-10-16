#pragma once

#include <common/KinematicController/RobotConfig.h>

class RegulatedMotor {
public:
  RegulatedMotor();

  bool isStopped();

  double runPid(double dt_s, double angle_rad);

  void setAcceleration(double acceleration);

  void setSetpoint(double setpoint);

  void reset_enc_rad(double rad);

  void setParams(double kP, double kI, double kD, double ff_offset, double int_cap);

  double kP;
  double kI;
  double kD;
  double ff_offset;
  double int_cap;

  bool initialized = false;
  double abstract_force;
  double acceleration_cellpss;
  double derivative;
  double error;
  double estimated_velocity_rps;
  double feed_forward;
  double integral;
  double last_angle_rad;
  double last_error;
  double last_velocity_rps;
  double regulated_setpoint_rps;
  double setpoint_rps;
  double smooth_derivative;
  double velocity_rps;
};