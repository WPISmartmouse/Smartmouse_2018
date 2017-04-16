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
  static const double ff_offset;
  static const double INTEGRAL_CAP;
  static const double DERIV_CAP;
  static const double MIN_ABSTRACT_FORCE;

  bool initialized = false;
  double abstract_force;
  double acceleration;
  double brake_acceleration;
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