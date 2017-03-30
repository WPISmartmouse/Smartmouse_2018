#pragma once

class RegulatedMotor {
public:
  RegulatedMotor(unsigned long period_ms);

  double run_pid(unsigned long time_ms, double angle_radians);

  void set_setpoint(double setpoint_rps);

  static constexpr double kP = 0.0;
  static constexpr double kI = 0.0;
  static constexpr double kD = 0.0;
  static constexpr double kFF = 0.0;
  const double INTEGRAL_CAP = 1.0;

  double abstract_force;
  double integral;
  double last_angle_rad;
  double last_error;
  double setpoint_rps;
  unsigned long period_ms;
  unsigned long last_update_time_ms;

};