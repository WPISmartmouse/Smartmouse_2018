#include <algorithm>
#include <common/Mouse.h>
#include "KinematicController.h"

KinematicMotorController::KinematicMotorController(unsigned long period_ms) : left_motor(period_ms),
                                                                              right_motor(period_ms) {}

Pose KinematicMotorController::get_pose() {
  return current_pose_estimate;
}

std::pair<double, double>
KinematicMotorController::run(unsigned long time_ms, double left_angle_rad, double right_angle_rad) {
  std::pair<double, double> abstract_forces;

  abstract_forces.first = left_motor.run_pid(time_ms, left_angle_rad);
  abstract_forces.second = right_motor.run_pid(time_ms, right_angle_rad);

  return abstract_forces;
}

void KinematicMotorController::setAcceleration(double start_acceleration, double stop_acceleration) {
  this->start_acceleration = start_acceleration;
  this->stop_acceleration = stop_acceleration;
}

void KinematicMotorController::setSpeed(double left_wheel_velocity_setpoint_mps,
                                        double right_wheel_velocity_setpoint_mps) {
  static double left_wheel_velocity_mps;
  static double right_wheel_velocity_mps;

  double left_acc = start_acceleration;
  double right_acc = start_acceleration;

  if (left_wheel_velocity_setpoint_mps == 0) {
    left_acc = stop_acceleration;
  }
  if (right_wheel_velocity_setpoint_mps == 0) {
    right_acc = stop_acceleration;
  }

  if (right_wheel_velocity_mps < right_wheel_velocity_setpoint_mps) {
    right_wheel_velocity_mps = std::min(right_wheel_velocity_mps + right_acc, right_wheel_velocity_setpoint_mps);
  } else if (right_wheel_velocity_mps > right_wheel_velocity_setpoint_mps) {
    right_wheel_velocity_mps = std::max(right_wheel_velocity_mps - right_acc, right_wheel_velocity_setpoint_mps);
  }

  if (left_wheel_velocity_mps < left_wheel_velocity_setpoint_mps) {
    left_wheel_velocity_mps = std::min(left_wheel_velocity_mps + left_acc, left_wheel_velocity_setpoint_mps);
  } else if (left_wheel_velocity_mps > left_wheel_velocity_setpoint_mps) {
    left_wheel_velocity_mps = std::max(left_wheel_velocity_mps - left_acc, left_wheel_velocity_setpoint_mps);
  }

  double left_wheel_velocity_rps = Mouse::metersPerSecToRadPerSec(left_wheel_velocity_mps);
  double right_wheel_velocity_rps = Mouse::metersPerSecToRadPerSec(right_wheel_velocity_mps);

  left_motor.set_setpoint(left_wheel_velocity_rps);
  right_motor.set_setpoint(right_wheel_velocity_rps);
}
