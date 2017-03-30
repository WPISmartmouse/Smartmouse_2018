#include <algorithm>
#include "RegulatedMotor.h"

RegulatedMotor::RegulatedMotor(unsigned long period_ms) : abstract_force(0), integral(0), last_angle_rad(0),
                                                          last_error(0),
                                                          setpoint_rps(0), period_ms(period_ms),
                                                          last_update_time_ms(0) {}

double RegulatedMotor::run_pid(unsigned long time_ms, double angle_radians) {
  if (time_ms - last_update_time_ms >= period_ms) {
    last_update_time_ms = time_ms;

    double velocity_rps = angle_radians - last_angle_rad;
    double error = setpoint_rps - velocity_rps;
    double derivative = error - last_error;
    integral += error * kI;
    integral = std::max(std::min(integral, INTEGRAL_CAP), -INTEGRAL_CAP);

    abstract_force = error * kP + integral * kI + derivative * kD + setpoint_rps * kFF;
  }

  return abstract_force;
}

void RegulatedMotor::set_setpoint(double setpoint_rps) {
  this->setpoint_rps = setpoint_rps;
}
