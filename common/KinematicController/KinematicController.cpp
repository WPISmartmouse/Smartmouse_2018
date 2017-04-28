#include <algorithm>
#include <common/Mouse.h>
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
  std::tie(est_yaw, offset) = estimate_pose(range_data, mouse);

//  static int i = 0;
//  if (i == 10) {
//    i = 0;
//    print("%i %i %0.3f, %0.3f, %0.3f %0.3f\r\n", row, col, getPose().x, getPose().y, getPose().yaw, offset);
//  }
//  i++;

  if (!ignore_sensor_pose_estimate) {
    current_pose_estimate.yaw = est_yaw;

    double d_wall_front = 0;
    bool wall_in_front = false;
    if (range_data.front < 0.07) {
      double yaw_error = WallFollower::yawDiff(current_pose_estimate.yaw, dir_to_yaw(mouse->getDir()));
      d_wall_front = cos(yaw_error) * range_data.front + config.FRONT_ANALOG_X;
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

std::pair<double, double> KinematicController::estimate_pose(RangeData range_data, Mouse *mouse) {
  static double last_front_left_analog_dist;
  static double last_front_right_analog_dist;
  static double last_back_left_analog_dist;
  static double last_back_right_analog_dist;
  std::pair<double, double> newest_estimate;

  double *yaw = &newest_estimate.first;
  double *offset = &newest_estimate.second;

  double d1x_l = cos(config.BACK_ANALOG_ANGLE) * range_data.back_left + config.BACK_SIDE_ANALOG_X;
  double d2x_l = cos(config.FRONT_ANALOG_ANGLE) * range_data.front_left + config.FRONT_SIDE_ANALOG_X;
  double d1y_l = sin(config.BACK_ANALOG_ANGLE) * range_data.back_left + config.BACK_SIDE_ANALOG_Y;
  double d2y_l = sin(config.FRONT_ANALOG_ANGLE) * range_data.front_left + config.FRONT_SIDE_ANALOG_Y;
  double currentYaw_l = -atan2(d2y_l - d1y_l, d2x_l - d1x_l);

  double d1x_r = cos(config.BACK_ANALOG_ANGLE) * range_data.back_right + config.BACK_SIDE_ANALOG_X;
  double d2x_r = cos(config.FRONT_ANALOG_ANGLE) * range_data.front_right + config.FRONT_SIDE_ANALOG_X;
  double d1y_r = sin(config.BACK_ANALOG_ANGLE) * range_data.back_right + config.BACK_SIDE_ANALOG_Y;
  double d2y_r = sin(config.FRONT_ANALOG_ANGLE) * range_data.front_right + config.FRONT_SIDE_ANALOG_Y;
  double currentYaw_r = atan2(d2y_r - d1y_r, d2x_r - d1x_r);

  double d_to_wall_left = (d2x_l * d1y_l - d2y_l * d1x_l) / sqrt(pow(d2y_l - d1y_l, 2) + pow(d2x_l - d1x_l, 2));
  double d_to_wall_right = (d2x_r * d1y_r - d2y_r * d1x_r) / sqrt(pow(d2y_r - d1y_r, 2) + pow(d2x_r - d1x_r, 2));
  bool sense_left_wall = range_data.front_left < config.SIDE_WALL_THRESHOLD &&
                         range_data.back_left < config.SIDE_WALL_THRESHOLD;
  bool sense_right_wall = range_data.front_right < config.SIDE_WALL_THRESHOLD &&
                          range_data.back_right < config.SIDE_WALL_THRESHOLD;

  // this logic checks for walls that are "falling off" or "falling on"
  // if the change in sensor distance is above some threashold, the wall is arriving or leaving
  // so we don't yet follow the wall
  double d_back_left = fabs(range_data.back_left - last_back_left_analog_dist);
  double d_back_right = fabs(range_data.back_right - last_back_right_analog_dist);
  if (d_back_left> config.WALL_CHANGED_THRESHOLD) {
    sense_left_wall = false;
  }
  if (d_back_right > config.WALL_CHANGED_THRESHOLD) {
    sense_right_wall = false;
  }

  double d_front_left = fabs(range_data.front_left - last_front_left_analog_dist);
  double d_front_right = fabs(range_data.front_right - last_front_right_analog_dist);
  if (d_front_left > config.WALL_CHANGED_THRESHOLD) {
    sense_left_wall = false;
  }
  if (d_front_right > config.WALL_CHANGED_THRESHOLD) {
    sense_right_wall = false;
  }

  // consider the "logical" state of walls AND actual range reading
  if (sense_right_wall && mouse->isWallInDirection(right_of_dir(mouse->getDir()))) { // wall is on right
    *offset = AbstractMaze::UNIT_DIST - d_to_wall_right - AbstractMaze::HALF_WALL_THICKNESS;
    *yaw = dir_to_yaw(mouse->getDir()) + currentYaw_r;
  } else if (sense_left_wall && mouse->isWallInDirection(left_of_dir(mouse->getDir()))) { // wall is on left
    *offset = d_to_wall_left + AbstractMaze::HALF_WALL_THICKNESS;
    *yaw = dir_to_yaw(mouse->getDir()) + currentYaw_l;
  } else { // we're too far from any walls, use our pose estimation
    *offset = WallFollower::dispToLeftEdge(mouse);
    *yaw = mouse->getPose().yaw;
  }

  last_front_left_analog_dist = range_data.front_left;
  last_front_right_analog_dist = range_data.front_right;
  last_back_left_analog_dist = range_data.back_left;
  last_back_right_analog_dist = range_data.back_right;

  return newest_estimate;
};

void KinematicController::setAcceleration(double acceleration, double break_acceleration) {
  left_motor.setAcceleration(acceleration, break_acceleration);
  right_motor.setAcceleration(acceleration, break_acceleration);
}

void KinematicController::setSpeedMps(double left_setpoint_mps,
                                      double right_setpoint_mps) {
  left_motor.setSetpointMps(left_setpoint_mps);
  right_motor.setSetpointMps(right_setpoint_mps);
}

double KinematicController::yawDiff(double y1, double y2) {
  double diff = y2 - y1;
  if (diff > M_PI) return diff - M_PI * 2;
  if (diff < -M_PI) return diff + M_PI * 2;
  return diff;
}
