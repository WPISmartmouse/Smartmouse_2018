#include <algorithm>
#include <common/Mouse.h>
#include "RegulatedMotor.h"

const double RegulatedMotor::kP = 7.0;
const double RegulatedMotor::kI = 0.0;
const double RegulatedMotor::kD = 0.0;
const double RegulatedMotor::kFF = 8.5;
const double RegulatedMotor::INTEGRAL_CAP = 0.0;

RegulatedMotor::RegulatedMotor() : initialized(false), abstract_force(0), acceleration(0), brake_acceleration(0),
                                   integral(0), last_angle_rad(0), last_error(0), raw_abstract_force(0),
                                   setpoint_rps(0), velocity_rps(0) {}

bool RegulatedMotor::isStopped() {
  return velocity_rps == 0 && abstract_force == 0;
}

double RegulatedMotor::runPid(double dt_s, double angle_rad) {
  if (!initialized) {
    initialized = true;
    last_angle_rad = angle_rad;
    return 0;
  }

  velocity_rps = (angle_rad - last_angle_rad) / dt_s;
  error = setpoint_rps - velocity_rps;
  derivative = (error - last_error) / dt_s;

  integral += error * dt_s * kI;
  if (last_error > 0 and error <= 0) { integral = 0; }
  if (last_error < 0 and error >= 0) { integral = 0; }
  integral = std::max(std::min(integral, INTEGRAL_CAP), -INTEGRAL_CAP);

  double cbrt_setpoint_rps = std::cbrt(setpoint_rps);
  raw_abstract_force = cbrt_setpoint_rps * kFF + error * kP + integral * kI + derivative * kD;

  // limit the change in abstract_force by the quantity += acc
  double acc = acceleration * dt_s;
  if (raw_abstract_force == 0) {
    acc = brake_acceleration * dt_s;
  }

  if (raw_abstract_force > abstract_force) {
    abstract_force = std::min(abstract_force + acc, raw_abstract_force);
  } else if (raw_abstract_force < abstract_force) {
    abstract_force = std::max(abstract_force - acc, raw_abstract_force);
  }

  abstract_force = std::max(std::min(255.0, abstract_force), -255.0);

  last_angle_rad = angle_rad;

  return abstract_force;
}

void RegulatedMotor::setAcceleration(double acceleration, double brake_acceleration) {
  this->acceleration = Mouse::metersPerSecToRadPerSec(acceleration);
  this->brake_acceleration = Mouse::metersPerSecToRadPerSec(brake_acceleration);
}

void RegulatedMotor::setSetpointMps(double setpoint_mps) {
  this->setpoint_rps = Mouse::metersPerSecToRadPerSec(setpoint_mps);
}
