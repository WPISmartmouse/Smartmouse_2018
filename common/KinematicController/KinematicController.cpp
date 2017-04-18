#include <algorithm>
#include <common/Mouse.h>
#include <common/util.h>
#include <common/WallFollower.h>
#include <tuple>
#include "KinematicController.h"

KinematicController::KinematicController(Mouse *mouse) : ignore_sensor_pose_estimate(false),
                                                         initialized(false), mouse(mouse) {
  current_pose_estimate.x = 0;
  current_pose_estimate.y = 0;
  current_pose_estimate.yaw = 0;
}

Pose KinematicController::getPose() {
  return current_pose_estimate;
}

std::pair<double, double> KinematicController::getWheelVelocities() {
  std::pair<double, double> vels;
  vels.first = left_motor.velocity_rps;
  vels.second = right_motor.velocity_rps;
  return vels;
};

bool KinematicController::isStopped() {
  return left_motor.isStopped() && left_motor.isStopped();
}

void KinematicController::reset_x_to(double new_x) {
  current_pose_estimate.x = new_x;
}

void KinematicController::reset_y_to(double new_y) {
  current_pose_estimate.y = new_y;
}

void KinematicController::reset_yaw_to(double new_yaw) {
  current_pose_estimate.yaw = new_yaw;
}

std::pair<double, double>
KinematicController::run(double dt_s, double left_angle_rad, double right_angle_rad, double ground_truth_left_vel_rps,
                         double ground_truth_right_vel_rps, RangeData range_data) {
  static std::pair<double, double> abstract_forces;
  static bool p = true; // FIXME: remove

  if (!initialized) {
    initialized = true;
    abstract_forces.first = 0;
    abstract_forces.second = 0;
    return abstract_forces;
  }

  // equations based on https://chess.eecs.berkeley.edu/eecs149/documentation/differentialDrive.pdf
  double vl = Mouse::radToMeters(left_motor.velocity_rps);
  double vr = Mouse::radToMeters(right_motor.velocity_rps);

  double w = (vr - vl) / config.TRACK_WIDTH;
  double r = config.TRACK_WIDTH / 2 * (vl + vr) / (vr - vl);
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

  row = (unsigned int) (current_pose_estimate.y / AbstractMaze::UNIT_DIST);
  col = (unsigned int) (current_pose_estimate.x / AbstractMaze::UNIT_DIST);
  row_offset_to_edge = fmod(current_pose_estimate.y, AbstractMaze::UNIT_DIST);
  col_offset_to_edge = fmod(current_pose_estimate.x, AbstractMaze::UNIT_DIST);

  // given odometry estimate, improve estimate using sensors, then update odometry estimate to match our best estimate
  Pose kc_pose = getPose();
  current_pose_estimate = kc_pose;
  double est_yaw, offset;
  std::tie(est_yaw, offset) = WallFollower::estimate_pose(range_data, mouse);

  if (!ignore_sensor_pose_estimate) {
    if (p) {
//      print("allowing estimating pose from rangefinders\n");
      p = false;
    }

    current_pose_estimate.yaw = est_yaw;

    double d_wall_front = 0;
    bool wall_in_front = false;
    if (range_data.front_analog < 0.08) {
      double yaw_error = WallFollower::yawDiff(current_pose_estimate.yaw, dir_to_yaw(mouse->getDir()));
      d_wall_front = cos(yaw_error) * range_data.front_analog + config.FRONT_ANALOG_X;
      wall_in_front = true;
    }

    switch (mouse->getDir()) {
      case Direction::N:
        current_pose_estimate.x = (col * AbstractMaze::UNIT_DIST) + offset;
        if (wall_in_front) {
          current_pose_estimate.y = (row * AbstractMaze::UNIT_DIST) + d_wall_front + AbstractMaze::HALF_WALL_THICKNESS;
        }
        break;
      case Direction::S:
        current_pose_estimate.x = (col + 1) * AbstractMaze::UNIT_DIST - offset;
        if (wall_in_front) {
          current_pose_estimate.y =
                  ((row + 1) * AbstractMaze::UNIT_DIST) - d_wall_front - AbstractMaze::HALF_WALL_THICKNESS;
        }
        break;
      case Direction::E:
        current_pose_estimate.y = (row * AbstractMaze::UNIT_DIST) + offset;
        if (wall_in_front) {
          current_pose_estimate.x =
                  ((col + 1) * AbstractMaze::UNIT_DIST) - d_wall_front - AbstractMaze::HALF_WALL_THICKNESS;
        }
        break;
      case Direction::W:
        current_pose_estimate.y = (row + 1) * AbstractMaze::UNIT_DIST - offset;
        if (wall_in_front) {
          current_pose_estimate.x = (col * AbstractMaze::UNIT_DIST) + d_wall_front + AbstractMaze::HALF_WALL_THICKNESS;
        }
        break;
      default:
        break;
    }
  } else {
    if (!p) {
//      print("Ignoring rangefinder pose estimate.\n");
      p = true;
    }
  }

  // run PID, which will update the velocities of the wheels
  abstract_forces.first = left_motor.runPid(dt_s, left_angle_rad, ground_truth_left_vel_rps);
  abstract_forces.second = right_motor.runPid(dt_s, right_angle_rad, ground_truth_right_vel_rps);

//  static int i = 0;
//  if (i == 15) {
//  print("%f, %f, %f, %f\r\n", Mouse::radToMeters(left_motor.regulated_setpoint_rps),
//        Mouse::radToMeters(left_motor.velocity_rps), Mouse::radToMeters(right_motor.regulated_setpoint_rps),
//        Mouse::radToMeters(right_motor.velocity_rps));
//    i = 0;
//  }
//  i += 1;

  return abstract_forces;
}

void KinematicController::setAcceleration(double acceleration, double break_acceleration) {
  left_motor.setAcceleration(acceleration, break_acceleration);
  right_motor.setAcceleration(acceleration, break_acceleration);
}

void KinematicController::setSpeedMps(double left_setpoint_mps,
                                      double right_setpoint_mps) {
  left_motor.setSetpointMps(left_setpoint_mps);
  right_motor.setSetpointMps(right_setpoint_mps);
}
