#include <algorithm>
#include <common/core/Mouse.h>
#include <common/KinematicController/RobotConfig.h>
#include <common/KinematicController/RegulatedMotor.h>
#include <common/math/math.h>

RegulatedMotor::RegulatedMotor()
    : kP(10.0),
      kI(0.04),
      kD(0.0),
      ff_offset(785),
      ff_scale(2.7),
      int_cap(50),
      initialized(false),
      abstract_force(0),
      acceleration_rpss(0),
      integral(0),
      last_angle_rad(0),
      last_error(0),
      last_velocity_rps(0),
      setpoint_rps(0),
      velocity_rps(0),
      second_derivative(0),
      last_derivative(0) {}

bool RegulatedMotor::isStopped() {
  bool stopped =
      fabs(smartmouse::kc::radToMeters(velocity_rps)) <= 0.001
          && fabs(abstract_force) <= smartmouse::kc::MIN_ABSTRACT_FORCE;
  return stopped;
}

double RegulatedMotor::runPid(double dt_s, double angle_rad) {
  if (!initialized) {
    initialized = true;
    last_angle_rad = angle_rad;
    return 0;
  }

  auto d_rad = smartmouse::math::yaw_diff(last_angle_rad, angle_rad);
  velocity_rps = d_rad / dt_s;
  error = setpoint_rps - velocity_rps;
  derivative = (last_velocity_rps - velocity_rps) / dt_s;
  integral += error * dt_s;
  integral = std::max(std::min(integral, int_cap), -int_cap);

  if (setpoint_rps < 0) {
    feed_forward = setpoint_rps * ff_scale - ff_offset;
  } else if (setpoint_rps > 0) {
    feed_forward = setpoint_rps * ff_scale + ff_offset;
  } else {
    feed_forward = 0;
  }
  abstract_force = (feed_forward) + (error * kP) + (integral * kI) + (derivative * kD);

  abstract_force = std::max(std::min(1023.0, abstract_force), -1023.0);

  last_error = error;
  last_angle_rad = angle_rad;
  last_velocity_rps = velocity_rps;

  return abstract_force;
}

void RegulatedMotor::setAccelerationCpss(double acceleration_cellpss) {
  this->acceleration_rpss = smartmouse::kc::cellsToRad(acceleration_cellpss);
}

#include <iostream>

void RegulatedMotor::setSetpointCps(double setpoint_cups) {
  double s = 0.0;
  if (setpoint_cups > 0.0) {
    s = std::max(std::min(setpoint_cups, smartmouse::kc::MAX_SPEED_CUPS), smartmouse::kc::MIN_SPEED_CUPS);
  } else if (setpoint_cups < 0.0) {
    s = std::min(std::max(setpoint_cups, -smartmouse::kc::MAX_SPEED_CUPS), -smartmouse::kc::MIN_SPEED_CUPS);
  }
  this->setpoint_rps = smartmouse::kc::cellsToRad(s);
}

void RegulatedMotor::setParams(double kP, double kI, double kD, double ff_scale, double ff_offset) {
  this->kP = kP;
  this->kI = kI;
  this->kD = kD;
  this->ff_scale = ff_scale;
  this->ff_offset = ff_offset;
}
