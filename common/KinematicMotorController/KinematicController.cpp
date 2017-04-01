#include <algorithm>
#include <common/Mouse.h>
#include "KinematicController.h"

KinematicMotorController::KinematicMotorController(unsigned long period_ms) : initialized(false), period_ms(period_ms),
                                                                              last_run_time_ms(0) {
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
  static double left_wheel_velocity_mps;
  static double right_wheel_velocity_mps;
  static std::pair<double, double> abstract_forces;

  if (!initialized) {
    initialized = true;
    last_run_time_ms = time_ms;
    abstract_forces.first = 0;
    abstract_forces.second = 0;
    return abstract_forces;
  }

  // equations based on https://chess.eecs.berkeley.edu/eecs149/documentation/differentialDrive.pdf
  double vl = Mouse::radPerSecToMetersPerSec(left_motor.velocity_rps);
  double vr = Mouse::radPerSecToMetersPerSec(right_motor.velocity_rps);

//  printf("%f %f\n", vl, vr);

  double w = (vr - vl) / Mouse::TRACK_WIDTH;
  double r = Mouse::TRACK_WIDTH / 2 * (vl + vr) / (vr - vl);
  double dt_s = (time_ms - last_run_time_ms) / 1000.0;
  double x = current_pose_estimate.x;
  double y = current_pose_estimate.y;
  double yaw = current_pose_estimate.yaw;

  if (std::isnan(r)) {
    // this means we're stopped, so ignore it
  }
  else if (std::isinf(r)) {
    // going perfectly straight is a special condition
    current_pose_estimate.x += dt_s * (vl + vr) / 2;
    current_pose_estimate.y += dt_s * (vl + vr) / 2;
  } else {
    current_pose_estimate.x = -cos(w * dt_s) * r * sin(yaw) + sin(w * dt_s) * r * cos(yaw) + x + r * sin(yaw);
    current_pose_estimate.y = -sin(w * dt_s) * r * sin(yaw) - cos(w * dt_s) * r * cos(yaw) + y + r * cos(yaw);
    yaw += w * dt_s;

    if (yaw < -M_PI) {
      yaw += M_PI * 2;
    }
    else if (yaw >= M_PI) {
      yaw -= M_PI * 2;
    }

    current_pose_estimate.yaw = yaw;
  }

  // handle acceleration and servicing motor PIDs
  double left_acc = start_acceleration;
  double right_acc = start_acceleration;

  if (left_setpoint_mps == 0) {
    left_acc = brake_acceleration;
  }
  if (right_setpoint_mps == 0) {
    right_acc = brake_acceleration;
  }

  if (right_wheel_velocity_mps < right_setpoint_mps) {
    right_wheel_velocity_mps = std::min(right_wheel_velocity_mps + right_acc, right_setpoint_mps);
  } else if (right_wheel_velocity_mps > right_setpoint_mps) {
    right_wheel_velocity_mps = std::max(right_wheel_velocity_mps - right_acc, right_setpoint_mps);
  }

  if (left_wheel_velocity_mps < left_setpoint_mps) {
    left_wheel_velocity_mps = std::min(left_wheel_velocity_mps + left_acc, left_setpoint_mps);
  } else if (left_wheel_velocity_mps > left_setpoint_mps) {
    left_wheel_velocity_mps = std::max(left_wheel_velocity_mps - left_acc, left_setpoint_mps);
  }

  double left_wheel_velocity_rps = Mouse::metersPerSecToRadPerSec(left_wheel_velocity_mps);
  double right_wheel_velocity_rps = Mouse::metersPerSecToRadPerSec(right_wheel_velocity_mps);

  left_motor.set_setpoint(left_wheel_velocity_rps);
  right_motor.set_setpoint(right_wheel_velocity_rps);

  // run PID, which will update the velocities of the wheels
  if (dt_s > 0) {
    abstract_forces.first = left_motor.run_pid(dt_s, left_angle_rad);
    abstract_forces.second = right_motor.run_pid(dt_s, right_angle_rad);
  }

  last_run_time_ms = time_ms;

  return abstract_forces;
}

void KinematicMotorController::setAcceleration(double start_acceleration, double break_acceleration) {
  this->start_acceleration = start_acceleration;
  this->brake_acceleration = break_acceleration;
}

void KinematicMotorController::setSpeed(double left_wheel_velocity_setpoint_mps,
                                        double right_wheel_velocity_setpoint_mps) {
  this->left_setpoint_mps = left_wheel_velocity_setpoint_mps;
  this->right_setpoint_mps = right_wheel_velocity_setpoint_mps;
}
