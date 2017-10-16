#include <cmath>
#include <algorithm>
#include <cmath>
#include <tuple>
#include <common/math/math.h>

#include <common/core/Mouse.h>
#include <common/KinematicController/KinematicController.h>

const double KinematicController::DROP_SAFETY = 0.8;
const double KinematicController::kPWall = 0.80;
const double KinematicController::kPYaw = 7.0;

KinematicController::KinematicController(Mouse *mouse) : enable_sensor_pose_estimate(false), enabled(true), initialized(false),
                                                         ignoring_left(false), ignoring_right(false), mouse(mouse),
                                                         d_until_left_drop(0), d_until_right_drop(0) {
  current_pose_estimate.col = 0;
  current_pose_estimate.row = 0;
  current_pose_estimate.yaw = 0;
}

GlobalPose KinematicController::getGlobalPose() {
  return current_pose_estimate;
}

LocalPose KinematicController::getLocalPose() {
  LocalPose local_pose_estimate;
  local_pose_estimate.yaw_from_straight = smartmouse::math::yawDiff(dir_to_yaw(mouse->getDir()), current_pose_estimate.yaw);
  switch (mouse->getDir()) {
    case Direction::N:
      local_pose_estimate.to_back = ceil(current_pose_estimate.row) - current_pose_estimate.row;
      local_pose_estimate.to_left = ceil(current_pose_estimate.col) - current_pose_estimate.col;
      break;
    case Direction::S:
      local_pose_estimate.to_back = current_pose_estimate.row - floor(current_pose_estimate.row);
      local_pose_estimate.to_left = current_pose_estimate.col - floor(current_pose_estimate.col);
      break;
    case Direction::E:
      local_pose_estimate.to_back = current_pose_estimate.col - floor(current_pose_estimate.col);
      local_pose_estimate.to_left = current_pose_estimate.row - floor(current_pose_estimate.row);
      break;
    case Direction::W:
      local_pose_estimate.to_back = ceil(current_pose_estimate.col) - current_pose_estimate.col;
      local_pose_estimate.to_left = ceil(current_pose_estimate.row) - current_pose_estimate.row;
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

void KinematicController::reset_col_to(double new_col) {
  current_pose_estimate.col = new_col;
}

void KinematicController::reset_row_to(double new_row) {
  current_pose_estimate.row = new_row;
}

void KinematicController::reset_yaw_to(double new_yaw) {
  current_pose_estimate.yaw = new_yaw;
}

std::pair<double, double>
KinematicController::run(double dt_s, double left_angle_rad, double right_angle_rad, RangeData range_data) {
  static std::pair<double, double> abstract_forces;

  if (!initialized) {
    initialized = true;
    abstract_forces.first = 0;
    abstract_forces.second = 0;
    return abstract_forces;
  }

  // save the cycle time so other functions can use is
  // this prevents us from passing dt all over the place
  this->dt_s = dt_s;

  if (enabled) {
    // equations based on docs/dynamics_model.pdf
    double vl = smartmouse::kc::radToMeters(left_motor.velocity_rps);
    double vr = smartmouse::kc::radToMeters(right_motor.velocity_rps);

    double dcol, drow, dtheta;
    std::tie(dcol, drow, dtheta) = forwardKinematics(vr, vl, current_pose_estimate.yaw, dt_s);
    current_pose_estimate.col += dcol;
    current_pose_estimate.row += drow;
    current_pose_estimate.yaw += dtheta;
    smartmouse::math::wrapAngleRadInPlace(&current_pose_estimate.yaw);

    row = (unsigned int) (current_pose_estimate.row);
    col = (unsigned int) (current_pose_estimate.col);

    // given odometry estimate, improve estimate using sensors, then update odometry estimate to match our best estimate
    // offset is measured from center wall line left of the robot (it's in the local pose frame)
    double est_yaw, offset;
    bool no_walls;
    std::tie(est_yaw, offset, no_walls) = estimate_pose(range_data, mouse);

    // only override if no one has explicitly set enable_sensor_pose_estimate to true
    if (enable_sensor_pose_estimate && !no_walls) {
      current_pose_estimate.yaw = est_yaw;

      double d_wall_front = 0;
      bool wall_in_front = false;
      if (range_data.front < 0.08) {
        double yaw_error = smartmouse::math::yawDiff(current_pose_estimate.yaw, dir_to_yaw(mouse->getDir()));
        d_wall_front = cos(yaw_error) * range_data.front + smartmouse::kc::FRONT_ANALOG_X;
        wall_in_front = true;
      }

      switch (mouse->getDir()) {
        case Direction::N:
          current_pose_estimate.col = col + offset;
          if (wall_in_front) {
            current_pose_estimate.row = row + d_wall_front + smartmouse::maze::HALF_WALL_THICKNESS_M;
          }
          break;
        case Direction::S:
          current_pose_estimate.col = col + 1 - offset;
          if (wall_in_front) {
            current_pose_estimate.row = row + 1 - d_wall_front - smartmouse::maze::HALF_WALL_THICKNESS_M;
          }
          break;
        case Direction::E:
          current_pose_estimate.row = row + offset;
          if (wall_in_front) {
            current_pose_estimate.col = col + 1 - d_wall_front - smartmouse::maze::HALF_WALL_THICKNESS_M;
          }
          break;
        case Direction::W:
          current_pose_estimate.row = row + 1 - offset;
          if (wall_in_front) {
            current_pose_estimate.col = col + d_wall_front + smartmouse::maze::HALF_WALL_THICKNESS_M;
          }
          break;
        default:
          break;
      }
    }

    // run PID, which will update the velocities of the wheels
    abstract_forces.first = left_motor.runPid(dt_s, left_angle_rad);
    abstract_forces.second = right_motor.runPid(dt_s, right_angle_rad);

    print("%0.3f\n", smartmouse::kc::radToMeters(left_motor.setpoint_rps));
  }
  else {
    abstract_forces.first = 0;
    abstract_forces.second = 0;
  }

  return abstract_forces;
}

std::tuple<double, double, double> KinematicController::forwardKinematics(double vl, double vr, double yaw, double dt) {
  double dcol, drow, dtheta;
  double w = (vr - vl) / smartmouse::kc::TRACK_WIDTH;
  double R = smartmouse::kc::TRACK_WIDTH * (vl + vr) / (2 * (vr - vl));

  if (fabs(vl) < 1e-5 && fabs(vr) < 1e-5) {
    // this means we're stopped, so ignore it
    dcol = 0;
    drow = 0;
    dtheta = 0;
  } else if (fabs(vr - vl) < 1e-5) {
    // going perfectly straight is a special condition
    dcol = dt * (vl + vr) / 2 * cos(yaw);
    drow = dt * (vl + vr) / 2 * sin(yaw);
    dtheta = 0;
  } else {
    double dtheta_about_icc = w * dt; //eq 11
    dcol = R * (sin(dtheta_about_icc + yaw) - sin(yaw)); //eq 28
    drow = -R * (cos(dtheta_about_icc + yaw) - cos(yaw)); //eq 29
    dtheta = w * dt;
  }

  return std::tuple<double, double, double>{dcol, drow, dtheta};
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

  double d1x_l = cos(smartmouse::kc::BACK_ANALOG_ANGLE) * range_data.back_left + smartmouse::kc::BACK_SIDE_ANALOG_X;
  double d2x_l = cos(smartmouse::kc::FRONT_ANALOG_ANGLE) * range_data.front_left + smartmouse::kc::FRONT_SIDE_ANALOG_X;
  double d1y_l = sin(smartmouse::kc::BACK_ANALOG_ANGLE) * range_data.back_left + smartmouse::kc::BACK_SIDE_ANALOG_Y;
  double d2y_l = sin(smartmouse::kc::FRONT_ANALOG_ANGLE) * range_data.front_left + smartmouse::kc::FRONT_SIDE_ANALOG_Y;
  double currentYaw_l = -atan2(d2y_l - d1y_l, d2x_l - d1x_l);

  double d1x_r = cos(smartmouse::kc::BACK_ANALOG_ANGLE) * range_data.back_right + smartmouse::kc::BACK_SIDE_ANALOG_X;
  double d2x_r = cos(smartmouse::kc::FRONT_ANALOG_ANGLE) * range_data.front_right + smartmouse::kc::FRONT_SIDE_ANALOG_X;
  double d1y_r = sin(smartmouse::kc::BACK_ANALOG_ANGLE) * range_data.back_right + smartmouse::kc::BACK_SIDE_ANALOG_Y;
  double d2y_r = sin(smartmouse::kc::FRONT_ANALOG_ANGLE) * range_data.front_right + smartmouse::kc::FRONT_SIDE_ANALOG_Y;
  double currentYaw_r = atan2(d2y_r - d1y_r, d2x_r - d1x_r);

  double d_to_wall_left = (d2x_l * d1y_l - d2y_l * d1x_l) / sqrt(pow(d2y_l - d1y_l, 2) + pow(d2x_l - d1x_l, 2));
  double d_to_wall_right = (d2x_r * d1y_r - d2y_r * d1x_r) / sqrt(pow(d2y_r - d1y_r, 2) + pow(d2x_r - d1x_r, 2));
  bool sense_left_wall = range_data.front_left < smartmouse::kc::SIDE_WALL_THRESHOLD &&
                         range_data.back_left < smartmouse::kc::SIDE_WALL_THRESHOLD;
  bool sense_right_wall = range_data.front_right < smartmouse::kc::SIDE_WALL_THRESHOLD &&
                          range_data.back_right < smartmouse::kc::SIDE_WALL_THRESHOLD;


  // check for walls that will fall off in the near future (geralds!)

  if (range_data.gerald_left > smartmouse::kc::GERALD_WALL_THRESHOLD) {
    d_until_left_drop = DROP_SAFETY * tan(smartmouse::kc::GERALD_ANGLE) * d_to_wall_left;
    sense_left_wall = false;
  }

  if (range_data.gerald_right > smartmouse::kc::GERALD_WALL_THRESHOLD) {
    d_until_right_drop = DROP_SAFETY * tan(smartmouse::kc::GERALD_ANGLE) * d_to_wall_right;
    sense_right_wall = false;
  }

  // this logic checks for walls that are "falling off" or "falling on"
  // if the change in sensor distance is above some threashold, the wall is arriving or leaving
  // so we don't yet follow the wall
  double d_back_left = fabs(range_data.back_left - last_back_left_analog_dist);
  double d_back_right = fabs(range_data.back_right - last_back_right_analog_dist);
  if (d_back_left > smartmouse::kc::WALL_CHANGED_THRESHOLD) {
    sense_left_wall = false;
  }
  if (d_back_right > smartmouse::kc::WALL_CHANGED_THRESHOLD) {
    sense_right_wall = false;
  }

  double d_front_left = fabs(range_data.front_left - last_front_left_analog_dist);
  double d_front_right = fabs(range_data.front_right - last_front_right_analog_dist);
  if (d_front_left > smartmouse::kc::WALL_CHANGED_THRESHOLD) {
    sense_left_wall = false;
  }
  if (d_front_right > smartmouse::kc::WALL_CHANGED_THRESHOLD) {
    sense_right_wall = false;
  }

  // consider the "logical" state of walls AND actual range reading
  if (sense_right_wall && mouse->isWallInDirection(right_of_dir(mouse->getDir()))) { // wall is on right
    *offset = smartmouse::maze::UNIT_DIST_M - d_to_wall_right - smartmouse::maze::HALF_WALL_THICKNESS_M;
    *yaw = dir_to_yaw(mouse->getDir()) + currentYaw_r;
    *ignore_walls = false;
  } else if (sense_left_wall && mouse->isWallInDirection(left_of_dir(mouse->getDir()))) { // wall is on left
    *offset = d_to_wall_left + smartmouse::maze::HALF_WALL_THICKNESS_M;
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
  this->acceleration_cellpss = acceleration_mpss;
  left_motor.setAcceleration(acceleration_mpss);
  right_motor.setAcceleration(acceleration_mpss);
}

void KinematicController::setSpeedMps(double left_setpoint_mps,
                                      double right_setpoint_mps) {
  left_motor.setSetpoint(left_setpoint_mps);
  right_motor.setSetpoint(right_setpoint_mps);
}

void KinematicController::start(GlobalPose start_pose, double goalDisp, double v_final) {
  drive_straight_state.disp = 0;
  drive_straight_state.dispError = goalDisp;
  drive_straight_state.goalDisp = goalDisp;
  drive_straight_state.start_pose = start_pose;
  drive_straight_state.forward_v = (left_motor.velocity_rps + right_motor.velocity_rps) /2;
  drive_straight_state.v_final = v_final;
}

void KinematicController::planTraj(Waypoints waypoints) {
  TrajectoryPlanner planner(waypoints);
  Eigen::Matrix<double, 10, 1> plan = planner.plan();
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
  double ramp_d = (pow(drive_straight_state.forward_v, 2)-pow(drive_straight_state.v_final, 2))/(2.0*acceleration_cellpss);
  double acc = acceleration_cellpss * dt_s;
  // TODO: this fudge factor is a mathematically justified hack.
  // it compensates for the use of discrete time
  if (drive_straight_state.dispError < ramp_d + 0.015) {
    drive_straight_state.forward_v -= acc;
  }
  else if (drive_straight_state.forward_v < smartmouse::kc::MAX_SPEED_CUPS) {
    drive_straight_state.forward_v += acc;
  }

  if (drive_straight_state.forward_v > smartmouse::kc::MAX_SPEED_CUPS) {
    drive_straight_state.forward_v = smartmouse::kc::MAX_SPEED_CUPS;
  }
  // TODO: this is problematic, at 180's we always stop and this is wrong
  else if (drive_straight_state.forward_v < drive_straight_state.v_final) {
     drive_straight_state.forward_v = drive_straight_state.v_final;
  }

  drive_straight_state.left_speed_cellps = drive_straight_state.forward_v;
  drive_straight_state.right_speed_cellps = drive_straight_state.forward_v;
  double correction = kPWall * yawError;

  if (yawError < 0) { // need to turn left
    drive_straight_state.left_speed_cellps += correction; // correction will be negative here
  } else {
    drive_straight_state.right_speed_cellps -= correction;
  }

  return std::pair<double, double>(drive_straight_state.left_speed_cellps, drive_straight_state.right_speed_cellps);
}

double KinematicController::yawDiff(double y1, double y2) {
  double diff = y2 - y1;
  if (diff > M_PI) return diff - M_PI * 2;
  if (diff < -M_PI) return diff + M_PI * 2;
  return diff;
}

double KinematicController::fwdDisp(Direction dir, GlobalPose current_pose, GlobalPose start_pose) {
  switch (dir) {
    case Direction::N:return start_pose.row - current_pose.row;
    case Direction::E:return current_pose.col - start_pose.col;
    case Direction::S:return current_pose.row - start_pose.row;
    case Direction::W:return start_pose.col - current_pose.col;
    default:return std::numeric_limits<double>::quiet_NaN();
  }
}

double KinematicController::dispToNextEdge(Mouse *mouse) {
  GlobalPose current_pose = mouse->getGlobalPose();
  Direction dir = mouse->getDir();

  switch (dir) {
    case Direction::N: {
      return current_pose.row - mouse->getRow();
    }
    case Direction::S: {
      return mouse->getRow() + 1 - current_pose.row;
    }
    case Direction::E: {
      return mouse->getCol() + 1 - current_pose.col;
    }
    case Direction::W: {
      return current_pose.col - mouse->getCol();
    }
    default:return std::numeric_limits<double>::quiet_NaN();
  }
}

double KinematicController::dispToNthEdge(Mouse *mouse, unsigned int n) {
  // give the displacement to the nth edge like above...
  return dispToNextEdge(mouse) + (n-1) * smartmouse::maze::UNIT_DIST_M;
}

GlobalPose KinematicController::poseOfToNthEdge(Mouse *mouse, unsigned int n) {
  // give the displacement to the nth edge like above...
  double disp = dispToNthEdge(mouse, n);
  GlobalPose pose = mouse->getGlobalPose();
  pose.col += cos(pose.yaw) * disp;
  pose.row += sin(pose.yaw) * disp;

  return pose;
}

double KinematicController::sidewaysDispToCenter(Mouse *mouse) {
  // local y is sideways, increasing from left to right
  return mouse->getLocalPose().to_left - smartmouse::maze::HALF_UNIT_DIST;
}

double KinematicController::fwdDispToCenter(Mouse *mouse) {
  return smartmouse::maze::HALF_UNIT_DIST - mouse->getLocalPose().to_back;
}

void KinematicController::setParams(double kP, double kI, double kD, double ff_offset, double int_cap) {
  left_motor.setParams(kP, kI, kD, ff_offset, int_cap);
  right_motor.setParams(kP, kI, kD, ff_offset, int_cap);
}
