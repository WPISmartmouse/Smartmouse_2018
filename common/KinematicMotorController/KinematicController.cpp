#include <algorithm>
#include <common/Mouse.h>
#include "KinematicController.h"

KinematicMotorController::KinematicMotorController(unsigned long period_ms) : left_motor(period_ms),
                                                                              right_motor(period_ms) {
  current_pose_estimate.x = 0;
  current_pose_estimate.y = 0;
  current_pose_estimate.yaw = 0;
}

Pose KinematicMotorController::get_pose() {
  return current_pose_estimate;
}

void KinematicMotorController::reset_x_to(double new_x) {
  current_pose_estimate.x = new_x;
}

void KinematicMotorController::reset_y_to(double new_y) {
  current_pose_estimate.y = new_y;
}

void KinematicMotorController::reset_yaw_to(double new_yaw) {
  current_pose_estimate.yaw = new_yaw;
}

std::pair<double, double>
KinematicMotorController::run(unsigned long time_ms, double left_angle_rad, double right_angle_rad) {
  static unsigned long last_run_time_ms = 0;
  static bool initialized = false;
  std::pair<double, double> abstract_forces;
  abstract_forces.first = 0;
  abstract_forces.second = 0;

  if (!initialized) {
    initialized = true;
    last_run_time_ms = time_ms;
    return abstract_forces;
  }

  // run PID, which will update the velocities of the wheels
  abstract_forces.first = left_motor.run_pid(time_ms, left_angle_rad);
  abstract_forces.second = right_motor.run_pid(time_ms, right_angle_rad);

  printf("%f\n", left_motor.smoothed_velocity_rps);

  double vl = left_motor.smoothed_velocity_rps;
  double vr = right_motor.smoothed_velocity_rps;
  double w = (vr - vl) / Mouse::TRACK_WIDTH;
  double r = Mouse::TRACK_WIDTH / 2 * (vl + vr) / (vr - vl);
  double dt = (time_ms - last_run_time_ms) / 1000.0;
  double x = current_pose_estimate.x;
  double y = current_pose_estimate.y;
  double yaw = current_pose_estimate.yaw;

  // going perfectly straight is a special condition
  if (std::isnan(r)) {
    current_pose_estimate.x += dt * (vl + vr) / 2;
    current_pose_estimate.y += dt * (vl + vr) / 2;
  }
  else {
    current_pose_estimate.x = cos(w * dt) * r * sin(yaw) + sin(w * dt) * r * cos(yaw) + x - r * sin(yaw);
    current_pose_estimate.y = sin(w * dt) * r * sin(yaw) - cos(w * dt) * r * cos(yaw) + y + r * cos(yaw);
    current_pose_estimate.yaw = yaw + w * dt;
  }

  last_run_time_ms = time_ms;

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
