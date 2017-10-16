#include <algorithm>
#include <common/core/Mouse.h>
#include <common/KinematicController/RobotConfig.h>
#include "RegulatedMotor.h"

RegulatedMotor::RegulatedMotor()
    : kP(6.5),
      kI(0.00),
      kD(0.2),
      ff_offset(15),
      int_cap(0),
      initialized(false),
      abstract_force(0),
      acceleration_cellpss(0),
      integral(0),
      last_angle_rad(0),
      last_error(0),
      last_velocity_rps(0),
      regulated_setpoint_rps(0),
      setpoint_rps(0),
      smooth_derivative(0),
      velocity_rps(0) {}

bool RegulatedMotor::isStopped() {
  bool stopped =
      fabs(smartmouse::kc::radToMeters(velocity_rps)) <= 0.001 && fabs(abstract_force) <= smartmouse::kc::MIN_ABSTRACT_FORCE;
  return stopped;
}

void RegulatedMotor::reset_enc_rad(double rad) {
  last_angle_rad = rad;
}

double RegulatedMotor::runPid(double dt_s, double angle_rad) {
  if (!initialized) {
    initialized = true;
    last_angle_rad = angle_rad;
    return 0;
  }

  estimated_velocity_rps = (angle_rad - last_angle_rad) / dt_s;
  velocity_rps = estimated_velocity_rps;
  error = regulated_setpoint_rps - velocity_rps;
  derivative = (last_velocity_rps - velocity_rps) / dt_s;
  smooth_derivative = 0.80 * smooth_derivative + 0.2 * derivative;
  integral += error * dt_s;
  integral = std::max(std::min(integral, int_cap), -int_cap);

  if (fabs(regulated_setpoint_rps) <= smartmouse::kc::MIN_SPEED) {
    abstract_force = 0;
  } else {
    // this is horrible and hacky
    // instead, we can use our model model to solve for the theoretical feed forward constant
    if (regulated_setpoint_rps < 0) {
      feed_forward = smartmouse::kc::radToMeters(regulated_setpoint_rps) * 100 - ff_offset;
    } else {
      feed_forward = smartmouse::kc::radToMeters(regulated_setpoint_rps) * 100 + ff_offset;
    }
    abstract_force = (feed_forward) + (error * kP) + (integral * kI) + (smooth_derivative * kD);
  }

  abstract_force = std::max(std::min(255.0, abstract_force), -255.0);

  // TODO remove this, since we have acceleration in KC
  // limit the change in setpoint
  double acc = acceleration_cellpss * dt_s;
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

void RegulatedMotor::setAcceleration(double acceleration_cellpss) {
  this->acceleration_cellpss = smartmouse::kc::cellsToRad(acceleration_cellpss);
}

void RegulatedMotor::setSetpoint(double setpoint_mps) {
  double s = 0;
  if (setpoint_mps > 0) {
    s = fmax(fmin(setpoint_mps, smartmouse::kc::MAX_SPEED), smartmouse::kc::MIN_SPEED);
  } else if (setpoint_mps < 0) {
    s = fmin(fmax(setpoint_mps, -smartmouse::kc::MAX_SPEED), -smartmouse::kc::MIN_SPEED);
  }
  this->setpoint_rps = smartmouse::kc::meterToRad(s);
}

void RegulatedMotor::setParams(double kP, double kI, double kD, double ff_offset, double int_cap) {
  this->kP = kP;
  this->kI = kI;
  this->kD = kD;
  this->ff_offset = ff_offset;
  this->int_cap = int_cap;
}
