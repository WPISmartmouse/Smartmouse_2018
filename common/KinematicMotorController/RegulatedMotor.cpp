#include <algorithm>
#include "RegulatedMotor.h"

const double RegulatedMotor::kP = 10;
const double RegulatedMotor::kI = 0.0;
const double RegulatedMotor::kD = 0.0;
const double RegulatedMotor::kFF = 0.0;
const double RegulatedMotor::INTEGRAL_CAP = 0.0;

RegulatedMotor::RegulatedMotor(unsigned long period_ms) : abstract_force(0), integral(0), last_angle_rad(0),
                                                          last_error(0),
                                                          setpoint_rps(0), period_ms(period_ms),
                                                          last_update_time_ms(0) {}

double RegulatedMotor::run_pid(unsigned long time_ms, double angle_rad) {
  auto dt = time_ms - last_update_time_ms;
  if (dt >= period_ms) {

    velocity_rps = (angle_rad - last_angle_rad) / (dt / 1000.0);
    error = setpoint_rps - velocity_rps;
    derivative = (error - last_error) / dt;
    integral += error * dt * kI;
    integral = std::max(std::min(integral, INTEGRAL_CAP), -INTEGRAL_CAP);

    abstract_force = setpoint_rps * kFF + error * kP + integral * kI + derivative * kD;

    abstract_force = std::max(std::min(255.0, abstract_force), -255.0);

    last_update_time_ms = time_ms;
    last_angle_rad = angle_rad;
  }


  return abstract_force;
}

void RegulatedMotor::set_setpoint(double setpoint_rps) {
  this->setpoint_rps = setpoint_rps;
}
