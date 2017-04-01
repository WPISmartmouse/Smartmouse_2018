#pragma once

class RegulatedMotor {
public:
  RegulatedMotor();

  double run_pid(double dt_s, double angle_rad);

  void set_setpoint(double setpoint_rps);

  static const double kP;
  static const double kI;
  static const double kD;
  static const double kFF;
  static const double INTEGRAL_CAP;
  static const double VEL_SMOOTHING;

  bool initialized = false;
  double abstract_force;
  double derivative;
  double error;
  double integral;
  double last_angle_rad;
  double last_error;
  double setpoint_rps;
  double smoothed_velocity_rps;
  double velocity_rps;
};