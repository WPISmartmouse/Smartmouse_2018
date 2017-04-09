#include <algorithm>
#include <common/Mouse.h>
#include "RegulatedMotor.h"

const double RegulatedMotor::kP = 6.0;
const double RegulatedMotor::kI = 0.00;
const double RegulatedMotor::kD = 0.1;
const double RegulatedMotor::ff_offset = 15;
const double RegulatedMotor::INTEGRAL_CAP = 0.0;
const double RegulatedMotor::DERIV_CAP = 0.0;
const double RegulatedMotor::MIN_ABSTRACT_FORCE = 3.5;

RegulatedMotor::RegulatedMotor() : initialized(false), abstract_force(0), acceleration(0), brake_acceleration(0),
                                   integral(0), last_angle_rad(0), last_error(0), last_velocity_rps(0),
                                   regulated_setpoint_rps(0), setpoint_rps(0),
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
#ifdef EMBED
  velocity_rps = estimated_velocity_rps;
#else
  velocity_rps = Mouse::meterToRad(ground_truth_velocity_mps);
#endif
  error = regulated_setpoint_rps - velocity_rps;
  derivative = (last_velocity_rps - velocity_rps) / dt_s;
  smooth_derivative = 0.80 * smooth_derivative + 0.2 * derivative;
  integral += error * dt_s;
  integral = std::max(std::min(integral, INTEGRAL_CAP), -INTEGRAL_CAP);

  if (fabs(regulated_setpoint_rps) <= Mouse::meterToRad(0.02)) {
    abstract_force = (error * kP) + (integral * kI) + (smooth_derivative * kD);
  }
  else {
    if (regulated_setpoint_rps < 0) {
      feed_forward = Mouse::radToMeters(regulated_setpoint_rps) * 100 - ff_offset;
    }
    else {
      feed_forward = Mouse::radToMeters(regulated_setpoint_rps) * 100 + ff_offset;
    }
    abstract_force = (feed_forward) + (error * kP) + (integral * kI) + (smooth_derivative * kD);
  }

  abstract_force = std::max(std::min(255.0, abstract_force), -255.0);

  // limit the change in setpoint
  double acc = acceleration * dt_s;
  if (abstract_force == 0) {
    acc = brake_acceleration * dt_s;
  }

  if (regulated_setpoint_rps < setpoint_rps) {
    regulated_setpoint_rps = std::min(regulated_setpoint_rps + acc, setpoint_rps);
  } else if (regulated_setpoint_rps > setpoint_rps) {
    regulated_setpoint_rps = std::max(regulated_setpoint_rps - acc, setpoint_rps);
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
