#include <algorithm>
#include <common/Mouse.h>
#include "RegulatedMotor.h"

const double RegulatedMotor::kP = 2.0;
const double RegulatedMotor::kI = 0.0;
const double RegulatedMotor::kD = 0.04;
const double RegulatedMotor::kFF = 1.5;
const double RegulatedMotor::INTEGRAL_CAP = 0.0;
const double RegulatedMotor::DERIV_CAP = 0.0;
const double RegulatedMotor::MIN_ERR = 0.2;

RegulatedMotor::RegulatedMotor() : initialized(false), abstract_force(0), acceleration(0), brake_acceleration(0),
                                   integral(0), last_angle_rad(0), last_error(0), last_velocity_rps(0),
                                   raw_abstract_force(0), regulated_setpoint_rps(0), setpoint_rps(0),
                                   smooth_derivative(0), velocity_rps(0) {}

bool RegulatedMotor::isStopped() {
  return velocity_rps == 0 && abstract_force == 0;
}

double RegulatedMotor::runPid(double dt_s, double angle_rad, double ground_truth_velocity_mps) {
  if (!initialized) {
    initialized = true;
    last_angle_rad = angle_rad;
    return 0;
  }

  estimated_velocity_rps = (angle_rad - last_angle_rad) / dt_s;
  velocity_rps = Mouse::meterToRad(ground_truth_velocity_mps);
  error = regulated_setpoint_rps - velocity_rps;
  derivative = (last_velocity_rps - velocity_rps) / dt_s;
  smooth_derivative = 0.80 * smooth_derivative + 0.2 * derivative;

  abstract_force = (regulated_setpoint_rps * kFF) + (error * kP) + (derivative * kD);
  abstract_force = std::fmax(std::fmin(255.0, abstract_force), -255.0);

  // limit the change in setpoint
  double acc = acceleration * dt_s;
  if (abstract_force == 0) {
    acc = brake_acceleration * dt_s;
  }

  if (regulated_setpoint_rps < setpoint_rps) {
    regulated_setpoint_rps = std::fmin(regulated_setpoint_rps + acc, setpoint_rps);
  } else if (regulated_setpoint_rps > setpoint_rps) {
    regulated_setpoint_rps = std::fmax(regulated_setpoint_rps - acc, setpoint_rps);
  }

  last_error = error;
  last_angle_rad = angle_rad;
  last_velocity_rps = velocity_rps;

  return abstract_force;
}

void RegulatedMotor::setAcceleration(double acceleration, double brake_acceleration) {
  this->acceleration = Mouse::meterToRad(acceleration);
  this->brake_acceleration = Mouse::meterToRad(brake_acceleration);
}

void RegulatedMotor::setSetpointMps(double setpoint_mps) {
  this->setpoint_rps = Mouse::meterToRad(setpoint_mps);
}
