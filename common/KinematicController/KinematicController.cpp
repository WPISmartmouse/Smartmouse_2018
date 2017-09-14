#include <algorithm>
#include <common/Mouse.h>
#include <tuple>
#include "KinematicController.h"
#include <cmath>
#include <common/commanduino/Command.h>

#ifdef EMBED
#include <Arduino.h>
#include <common/commanduino/Command.h>
#endif

const double KinematicController::DROP_SAFETY = 0.8;
const double KinematicController::POST_DROP_DIST = 0.05;
const double KinematicController::kPWall = 0.80;
const double KinematicController::kDWall = 50;
const double KinematicController::kPYaw = 7.0;

KinematicController::KinematicController(Mouse *mouse) : enable_sensor_pose_estimate(false), enabled(true), initialized(false),
                                                         ignoring_left(false), ignoring_right(false), mouse(mouse),
                                                         d_until_left_drop(0), d_until_right_drop(0) {
  current_pose_estimate.x = 0;
  current_pose_estimate.y = 0;
  current_pose_estimate.yaw = 0;
}

GlobalPose KinematicController::getGlobalPose() {
  return current_pose_estimate;
}

LocalPose KinematicController::getLocalPose() {
  LocalPose local_pose_estimate;
  local_pose_estimate.yaw_from_straight = yawDiff(dir_to_yaw(mouse->getDir()), current_pose_estimate.yaw);
  switch (mouse->getDir()) {
    case Direction::N:
      local_pose_estimate.to_back = (row + 1) * AbstractMaze::UNIT_DIST - current_pose_estimate.y;
      local_pose_estimate.to_left = current_pose_estimate.x - col * AbstractMaze::UNIT_DIST;
      break;
    case Direction::S:
      local_pose_estimate.to_back = current_pose_estimate.y - row * AbstractMaze::UNIT_DIST;
      local_pose_estimate.to_left = (col + 1) * AbstractMaze::UNIT_DIST - current_pose_estimate.x;
      break;
    case Direction::E:
      local_pose_estimate.to_left = current_pose_estimate.y - row * AbstractMaze::UNIT_DIST;
      local_pose_estimate.to_back = current_pose_estimate.x - col * AbstractMaze::UNIT_DIST;
      break;
    case Direction::W:
      local_pose_estimate.to_left = (row + 1) * AbstractMaze::UNIT_DIST - current_pose_estimate.y;
      local_pose_estimate.to_back = (col + 1) * AbstractMaze::UNIT_DIST - current_pose_estimate.x;
      break;
    default:
      exit(-1);
  }
  return local_pose_estimate;
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

  if (enabled) {
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
    double est_yaw, offset;
    bool no_walls;
    std::tie(est_yaw, offset, no_walls) = estimate_pose(range_data, mouse);

    // only override if no on was explicitly set enable_sensor_pose_estimate to true
    if (enable_sensor_pose_estimate && !no_walls) {
      current_pose_estimate.yaw = est_yaw;

      double d_wall_front = 0;
      bool wall_in_front = false;
      if (range_data.front < 0.08) {
        double yaw_error = KinematicController::yawDiff(current_pose_estimate.yaw, dir_to_yaw(mouse->getDir()));
        d_wall_front = cos(yaw_error) * range_data.front + config.FRONT_ANALOG_X;
        wall_in_front = true;
      }

      switch (mouse->getDir()) {
        case Direction::N:
          current_pose_estimate.x = (col * AbstractMaze::UNIT_DIST) + offset;
          if (wall_in_front) {
            current_pose_estimate.y =
                    (row * AbstractMaze::UNIT_DIST) + d_wall_front + AbstractMaze::HALF_WALL_THICKNESS;
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
            current_pose_estimate.x =
                    (col * AbstractMaze::UNIT_DIST) + d_wall_front + AbstractMaze::HALF_WALL_THICKNESS;
          }
          break;
        default:
          break;
      }
    }

    // run PID, which will update the velocities of the wheels
    abstract_forces.first = left_motor.runPid(dt_s, left_angle_rad, ground_truth_left_vel_rps);
    abstract_forces.second = right_motor.runPid(dt_s, right_angle_rad, ground_truth_right_vel_rps);
  }
  else {
    abstract_forces.first = 0;
    abstract_forces.second = 0;
  }

  return abstract_forces;
}

std::tuple<double, double, bool> KinematicController::estimate_pose(RangeData range_data, Mouse *mouse) {
  static double last_front_left_analog_dist;
  static double last_front_right_analog_dist;
  static double last_back_left_analog_dist;
  static double last_back_right_analog_dist;
  std::tuple<double, double, bool> newest_estimate;

  double *yaw = &std::get<0>(newest_estimate);
  double *offset = &std::get<1>(newest_estimate);
  bool *ignore_walls = &std::get<2>(newest_estimate);

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


  // check for walls that will fall off in the near future (geralds!)

  if (range_data.gerald_left > config.GERALD_WALL_THRESHOLD) {
    d_until_left_drop = DROP_SAFETY * tan(config.GERALD_ANGLE) * d_to_wall_left;
    sense_left_wall = false;
  }

  if (range_data.gerald_right > config.GERALD_WALL_THRESHOLD) {
    d_until_right_drop = DROP_SAFETY * tan(config.GERALD_ANGLE) * d_to_wall_right;
    sense_right_wall = false;
  }

  // this logic checks for walls that are "falling off" or "falling on"
  // if the change in sensor distance is above some threashold, the wall is arriving or leaving
  // so we don't yet follow the wall
  double d_back_left = fabs(range_data.back_left - last_back_left_analog_dist);
  double d_back_right = fabs(range_data.back_right - last_back_right_analog_dist);
  if (d_back_left > config.WALL_CHANGED_THRESHOLD) {
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
    *ignore_walls = false;
  } else if (sense_left_wall && mouse->isWallInDirection(left_of_dir(mouse->getDir()))) { // wall is on left
    *offset = d_to_wall_left + AbstractMaze::HALF_WALL_THICKNESS;
    *yaw = dir_to_yaw(mouse->getDir()) + currentYaw_l;
    *ignore_walls = false;
  } else { // we're too far from any walls, use our pose estimation
    *ignore_walls = true;
  }

  last_front_left_analog_dist = range_data.front_left;
  last_front_right_analog_dist = range_data.front_right;
  last_back_left_analog_dist = range_data.back_left;
  last_back_right_analog_dist = range_data.back_right;

  return newest_estimate;
};

void KinematicController::setAccelerationMpss(double acceleration_mpss) {
  this->acceleration_mpss = acceleration_mpss;
  left_motor.setAccelerationMpss(acceleration_mpss);
  right_motor.setAccelerationMpss(acceleration_mpss);
}

void KinematicController::setSpeedMps(double left_setpoint_mps,
                                      double right_setpoint_mps) {
  left_motor.setSetpointMps(left_setpoint_mps);
  right_motor.setSetpointMps(right_setpoint_mps);
}

void KinematicController::start(GlobalPose start_pose, double goalDisp) {
  drive_straight_state.disp = 0;
  drive_straight_state.dispError = goalDisp;
  drive_straight_state.goalDisp = goalDisp;
  drive_straight_state.start_pose = start_pose;
  drive_straight_state.start_time_s = ((double)Command::getTimerImplementation()->programTimeMs()) / 1000;
}

std::pair<double, double> KinematicController::compute_wheel_velocities(Mouse *mouse) {
  GlobalPose current_pose = mouse->getGlobalPose();
  drive_straight_state.disp = fwdDisp(mouse->getDir(), current_pose, drive_straight_state.start_pose);
  drive_straight_state.dispError = drive_straight_state.goalDisp - drive_straight_state.disp;

  double errorToCenter = sidewaysDispToCenter(mouse);
  double goalYaw = dir_to_yaw(mouse->getDir()) + errorToCenter * kPYaw;

  // The goal is to be facing straight when you wall distance is correct.
  // To achieve this, we control our yaw as a function of our error in wall distance
  double yawError = KinematicController::yawDiff(goalYaw, current_pose.yaw);

  // given starting velocity, fixed acceleration, and final velocity
  // generate the velocity profile for achieving this as fast as possible
  double t = ((double)Command::getTimerImplementation()->programTimeMs()) / 1000;
  double dt_s = t - drive_straight_state.start_time_s;
  const double vf_mps = 0.20; // TODO: make this a function of what motion primitive comes next
  double t_max = 0; // FIXME
  if (dt_s < t_max) {
    if (drive_straight_state.left_speed_mps < config.MAX_SPEED) {
      drive_straight_state.left_speed_mps += acceleration_mpss;
    }
    else {
      drive_straight_state.left_speed_mps = config.MAX_SPEED;
    }
    if (drive_straight_state.right_speed_mps < config.MAX_SPEED) {
      drive_straight_state.right_speed_mps += acceleration_mpss;
    }
    else {
      drive_straight_state.left_speed_mps = config.MAX_SPEED;
    }
  }
  else {
    if (drive_straight_state.left_speed_mps > vf) {
      drive_straight_state.left_speed_mps -= acceleration_mpss;
    }
    else {
      drive_straight_state.left_speed_mps = vf;
    }
    if (drive_straight_state.right_speed_mps > vf) {
      drive_straight_state.right_speed_mps -= acceleration_mpss;
    }
    else {
      drive_straight_state.left_speed_mps = vf;
    }
  }

  double correction = kPWall * yawError;

  if (yawError < 0) { // need to turn left
    drive_straight_state.left_speed_mps += correction; // correction will be negative here
  } else {
    drive_straight_state.right_speed_mps -= correction;
  }

  return std::pair<double, double>(drive_straight_state.left_speed_mps, drive_straight_state.right_speed_mps);
}

double KinematicController::yawDiff(double y1, double y2) {
  double diff = y2 - y1;
  if (diff > M_PI) return diff - M_PI * 2;
  if (diff < -M_PI) return diff + M_PI * 2;
  return diff;
}

double KinematicController::fwdDisp(Direction dir, GlobalPose current_pose, GlobalPose start_pose) {
  switch (dir) {
    case Direction::N:return start_pose.y - current_pose.y;
    case Direction::E:return current_pose.x - start_pose.x;
    case Direction::S:return current_pose.y - start_pose.y;
    case Direction::W:return start_pose.x - current_pose.x;
    default:return std::numeric_limits<double>::quiet_NaN();
  }
}

double KinematicController::dispToNextEdge(Mouse *mouse) {
  GlobalPose current_pose = mouse->getGlobalPose();
  Direction dir = mouse->getDir();

  switch (dir) {
    case Direction::N: {
      double next_row_y = mouse->getRow() * AbstractMaze::UNIT_DIST;
      return current_pose.y - next_row_y;
    }
    case Direction::S: {
      double next_row_y = (mouse->getRow() + 1) * AbstractMaze::UNIT_DIST;
      return next_row_y - current_pose.y;
    }
    case Direction::E: {
      double next_col_x = (mouse->getCol() + 1) * AbstractMaze::UNIT_DIST;
      return next_col_x - current_pose.x;
    }
    case Direction::W: {
      double next_col_x = mouse->getCol() * AbstractMaze::UNIT_DIST;
      return current_pose.x - next_col_x;
    }
    default:return std::numeric_limits<double>::quiet_NaN();
  }
}

double KinematicController::dispToNthEdge(Mouse *mouse, unsigned int n) {
  // give the displacement to the nth edge like above...
  return 0;
}

double KinematicController::sidewaysDispToCenter(Mouse *mouse) {
  // local y is sideways, increasing from left to right
  return mouse->getLocalPose().to_left - AbstractMaze::HALF_UNIT_DIST;
}

double KinematicController::fwdDispToCenter(Mouse *mouse) {
  return AbstractMaze::HALF_UNIT_DIST - mouse->getLocalPose().to_back;
}

double KinematicController::fwdDispToDiag(Mouse *mouse) {
  return (AbstractMaze::HALF_UNIT_DIST - (config.TRACK_WIDTH / 2.0)) - mouse->getLocalPose().to_back;
}
