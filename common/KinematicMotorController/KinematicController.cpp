#include <algorithm>
#include <common/Mouse.h>
#include "KinematicController.h"

KinematicMotorController::KinematicMotorController() : initialized(false), last_run_time_ms(0) {
  current_pose_estimate.x = 0;
  current_pose_estimate.y = 0;
  current_pose_estimate.yaw = 0;
}

Pose KinematicMotorController::getPose() {
  return current_pose_estimate;
}

std::pair<double, double> KinematicMotorController::getWheelVelocities() {
  std::pair<double, double> vels;
  vels.first = left_motor.velocity_rps;
  vels.second = right_motor.velocity_rps;
  return vels;
};

bool KinematicMotorController::isStopped() {
  return left_motor.isStopped() && left_motor.isStopped();
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

  double w = (vr - vl) / Mouse::TRACK_WIDTH;
  double r = Mouse::TRACK_WIDTH / 2 * (vl + vr) / (vr - vl);
  double dt_s = (time_ms - last_run_time_ms) / 1000.0;
  double x = current_pose_estimate.x;
  double y = current_pose_estimate.y;
  double yaw = current_pose_estimate.yaw;

  if (std::isnan(r)) {
    // this means we're stopped, so ignore it
  } else if (std::isinf(r)) {
    // going perfectly straight is a special condition
    current_pose_estimate.x += (dt_s * (vl + vr) / 2) * cos(yaw);
    current_pose_estimate.y += -(dt_s * (vl + vr) / 2) * sin(yaw);
  } else {
    current_pose_estimate.x = -cos(w * dt_s) * r * sin(yaw) + sin(w * dt_s) * r * cos(yaw) + x + r * sin(yaw);
    current_pose_estimate.y = -sin(w * dt_s) * r * sin(yaw) - cos(w * dt_s) * r * cos(yaw) + y + r * cos(yaw);
    yaw += w * dt_s;

    if (yaw < -M_PI) {
      yaw += M_PI * 2;
    } else if (yaw >= M_PI) {
      yaw -= M_PI * 2;
    }

    current_pose_estimate.yaw = yaw;
  }

  // run PID, which will update the velocities of the wheels
  if (dt_s > 0) {
    abstract_forces.first = left_motor.runPid(dt_s, left_angle_rad);
    abstract_forces.second = right_motor.runPid(dt_s, right_angle_rad);
  }

  last_run_time_ms = time_ms;

  return abstract_forces;
}

void KinematicMotorController::setAcceleration(double acceleration, double break_acceleration) {
  left_motor.setAcceleration(acceleration, break_acceleration);
  right_motor.setAcceleration(acceleration, break_acceleration);
}

void KinematicMotorController::setSpeedMps(double left_setpoint_mps,
                                           double right_setpoint_mps) {
  left_motor.setSetpointMps(left_setpoint_mps);
  right_motor.setSetpointMps(right_setpoint_mps);
}
