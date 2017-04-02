#include <algorithm>
#include <common/Mouse.h>
#include "RegulatedMotor.h"

const double RegulatedMotor::kP = 2.0;
const double RegulatedMotor::kI = 0.8;
const double RegulatedMotor::kD = 0.08;
const double RegulatedMotor::kFF = 2.8;
const double RegulatedMotor::INTEGRAL_CAP = 10.0;
const double RegulatedMotor::DERIV_CAP = 0.0;
const double RegulatedMotor::MIN_ABSTRACT_FORCE = 4.5;

RegulatedMotor::RegulatedMotor() : initialized(false), abstract_force(0), acceleration(0), brake_acceleration(0),
                                   integral(0), last_angle_rad(0), last_error(0), last_velocity_rps(0),
                                   raw_abstract_force(0), regulated_setpoint_rps(0), setpoint_rps(0),
                                   smooth_derivative(0), velocity_rps(0) {}

bool RegulatedMotor::isStopped() {
  bool stopped =
          fabs(Mouse::radToMeters(velocity_rps)) <= 0.001 && fabs(abstract_force) <= RegulatedMotor::MIN_ABSTRACT_FORCE;
  return stopped;
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
  integral += error * dt_s;
  integral = std::fmax(std::fmin(integral, INTEGRAL_CAP), -INTEGRAL_CAP);

  double sqrt_setpoint = regulated_setpoint_rps > 0 ? sqrt(regulated_setpoint_rps) : -sqrt(-regulated_setpoint_rps);
  abstract_force = (sqrt_setpoint * kFF) + (error * kP) + (integral * kI) + (smooth_derivative * kD);
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
