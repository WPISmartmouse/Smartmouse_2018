#pragma once

class RegulatedMotor {
public:
  RegulatedMotor();

  bool isStopped();

  double runPid(double dt_s, double angle_rad, double ground_truth_velocity_mps);

  void setAcceleration(double acceleration, double brake_acceleration);

  void setSetpointMps(double setpoint_mps);

  static const double kP;
  static const double kI;
  static const double kD;
  static const double kFF;
  static const double INTEGRAL_CAP;
  static const double DERIV_CAP;
  static const double MIN_ERR;

  bool initialized = false;
  double abstract_force;
  double acceleration;
  double brake_acceleration;
  double regulated_setpoint_rps;
  double derivative;
  double error;
  double estimated_velocity_rps;
  double integral;
  double last_angle_rad;
  double last_error;
  double smooth_derivative;
  double last_velocity_rps;
  double raw_abstract_force;
  double setpoint_rps;
  double velocity_rps;
};