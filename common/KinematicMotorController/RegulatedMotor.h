#pragma once

class RegulatedMotor {
public:
  RegulatedMotor();

  bool isStopped();

  double runPid(double dt_s, double angle_rad);

  void setAcceleration(double acceleration, double brake_acceleration);

  void setSetpointMps(double setpoint_mps);

  static const double kP;
  static const double kI;
  static const double kD;
  static const double kFF;
  static const double INTEGRAL_CAP;

  bool initialized = false;
  double abstract_force;
  double acceleration;
  double brake_acceleration;
  double derivative;
  double error;
  double integral;
  double last_angle_rad;
  double last_error;
  double raw_abstract_force;
  double setpoint_rps;
  double velocity_rps;
};