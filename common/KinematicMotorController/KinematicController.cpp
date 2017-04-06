#include <algorithm>
#include <common/Mouse.h>
#include <common/util.h>
#include "KinematicController.h"

KinematicMotorController::KinematicMotorController() : initialized(false) {
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
KinematicMotorController::run(double dt_s, double left_angle_rad, double right_angle_rad,
                              double ground_truth_left_vel_rps, double ground_truth_right_vel_rps) {
  static std::pair<double, double> abstract_forces;

  if (!initialized) {
    initialized = true;
    abstract_forces.first = 0;
    abstract_forces.second = 0;
    return abstract_forces;
  }

  // equations based on https://chess.eecs.berkeley.edu/eecs149/documentation/differentialDrive.pdf
  double vl = Mouse::radToMeters(left_motor.velocity_rps);
  double vr = Mouse::radToMeters(right_motor.velocity_rps);

  double w = (vr - vl) / Mouse::TRACK_WIDTH;
  double r = Mouse::TRACK_WIDTH / 2 * (vl + vr) / (vr - vl);
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
  abstract_forces.first = left_motor.runPid(dt_s, left_angle_rad, ground_truth_left_vel_rps);
  abstract_forces.second = right_motor.runPid(dt_s, right_angle_rad, ground_truth_right_vel_rps);

  static int i = 0;
  if (i == 15) {
    print("%f, %f, %f, %f\n", left_motor.regulated_setpoint_rps, left_motor.velocity_rps,
          right_motor.regulated_setpoint_rps, right_motor.velocity_rps);
    i = 0;
  }
  i += 1;

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
