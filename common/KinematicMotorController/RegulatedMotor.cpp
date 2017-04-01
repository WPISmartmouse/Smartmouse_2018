#include <algorithm>
#include <common/Mouse.h>
#include "RegulatedMotor.h"

const double RegulatedMotor::kP = 30.0;
const double RegulatedMotor::kI = 0.0;
const double RegulatedMotor::kD = 0.0;
const double RegulatedMotor::kFF = 10.0;
const double RegulatedMotor::INTEGRAL_CAP = 0.0;
const double RegulatedMotor::VEL_SMOOTHING = 0.7;

RegulatedMotor::RegulatedMotor() : initialized(false), abstract_force(0), integral(0), last_angle_rad(0), last_error(0),
                                   setpoint_rps(0), smoothed_velocity_rps(0) {}

double RegulatedMotor::run_pid(unsigned long dt, double angle_rad) {
  if (!initialized) {
    initialized = true;
    last_angle_rad = angle_rad;
    return 0;
  }

  velocity_rps = (angle_rad - last_angle_rad) / (dt / 1000.0);
  smoothed_velocity_rps = (VEL_SMOOTHING * smoothed_velocity_rps) + ((1 - VEL_SMOOTHING) * velocity_rps);
  error = setpoint_rps - velocity_rps;
  derivative = (error - last_error) / dt;

  integral += error * dt * kI;
  if (last_error > 0 and error <= 0) { integral = 0;}
  if (last_error < 0 and error >= 0) { integral = 0;}
  integral = std::max(std::min(integral, INTEGRAL_CAP), -INTEGRAL_CAP);

  double sqrt_setpoint_rps;
  if (setpoint_rps < 0) {
    sqrt_setpoint_rps = -sqrt(-setpoint_rps);
  }
  else {
    sqrt_setpoint_rps = sqrt(setpoint_rps);
  }

  abstract_force = sqrt_setpoint_rps * kFF + error * kP + integral * kI + derivative * kD;

  double s_mps = Mouse::radPerSecToMetersPerSec(setpoint_rps);
  double v_mps = Mouse::radPerSecToMetersPerSec(velocity_rps);

  abstract_force = std::max(std::min(255.0, abstract_force), -255.0);

  last_angle_rad = angle_rad;

  return abstract_force;
}

void RegulatedMotor::set_setpoint(double setpoint_rps) {
  this->setpoint_rps = setpoint_rps;
}
