#include <tuple>
#include <limits>
#include "WallFollower.h"

const double WallFollower::kPWall = 0.1; //TODO: Should be 0.8
const double WallFollower::kDWall = 50;
const double WallFollower::kPYaw = 3.0;

WallFollower::WallFollower() : disp(0.0), goalDisp(AbstractMaze::UNIT_DIST), dispError(goalDisp) {}

WallFollower::WallFollower(double goalDisp) : disp(0.0), goalDisp(goalDisp), dispError(goalDisp) {}

std::pair<double, double> WallFollower::compute_wheel_velocities(Mouse *mouse, Pose start_pose, RangeData range_data) {
  Pose current_pose = mouse->getPose();
  disp = forwardDisplacement(mouse->getDir(), start_pose, current_pose);
  dispError = goalDisp - disp;

  double currentYaw, offset;
  std::tie(currentYaw, offset) = WallFollower::estimate_pose(range_data, mouse);
  double errorToCenter = offset - AbstractMaze::HALF_UNIT_DIST;

  double goalYawOffset = errorToCenter * kPYaw;

  double goalYaw = dir_to_yaw(mouse->getDir()) + goalYawOffset;

  // The goal is to be facing straight when you wall distance is correct.
  // To achieve this, we control our yaw as a function of our error in wall distance
  double yawError = yawDiff(goalYaw, currentYaw);

  double l = config.MAX_SPEED;
  double r = config.MAX_SPEED;
  double correction = kPWall * yawError; // in m/s

  if (yawError < 0) { // need to turn left
    l += correction; // correction will be negative here
  } else {
    r -= correction;
  }

  return std::pair<double, double>(l, r);
}

std::pair<double, double> WallFollower::estimate_pose(RangeData range_data, Mouse *mouse) {
  std::pair<double, double> out;
  double *yaw = &out.first;
  double *offset = &out.second;

  double d1x_l = cos(config.BACK_ANALOG_ANGLE) * range_data.back_left_analog + config.BACK_SIDE_ANALOG_X;
  double d2x_l = cos(config.FRONT_ANALOG_ANGLE) * range_data.front_left_analog + config.FRONT_SIDE_ANALOG_X;
  double d1y_l = sin(config.BACK_ANALOG_ANGLE) * range_data.back_left_analog + config.BACK_SIDE_ANALOG_Y;
  double d2y_l = sin(config.FRONT_ANALOG_ANGLE) * range_data.front_left_analog + config.FRONT_SIDE_ANALOG_Y;
  double currentYaw_l = -atan2(d2y_l - d1y_l, d2x_l- d1x_l);

  double d1x_r = cos(config.BACK_ANALOG_ANGLE) * range_data.back_right_analog + config.BACK_SIDE_ANALOG_X;
  double d2x_r = cos(config.FRONT_ANALOG_ANGLE) * range_data.front_right_analog + config.FRONT_SIDE_ANALOG_X;
  double d1y_r = sin(config.BACK_ANALOG_ANGLE) * range_data.back_right_analog + config.BACK_SIDE_ANALOG_Y;
  double d2y_r = sin(config.FRONT_ANALOG_ANGLE) * range_data.front_right_analog + config.FRONT_SIDE_ANALOG_Y;
  double currentYaw_r = atan2(d2y_r - d1y_r, d2x_r- d1x_r);

  double dToWallLeft = (d2x_l * d1y_l - d2y_l * d1x_l) / sqrt(pow(d2y_l - d1y_l, 2) + pow(d2x_l-d1x_l, 2));
  double dToWallRight = (d2x_r * d1y_r - d2y_r * d1x_r) / sqrt(pow(d2y_r - d1y_r, 2) + pow(d2x_r-d1x_r, 2));

  bool sense_left_wall = range_data.front_left_analog < config.WALL_THRESHOLD && range_data.back_left_analog < config.WALL_THRESHOLD;
  bool sense_right_wall = range_data.front_right_analog < config.WALL_THRESHOLD && range_data.back_right_analog < config.WALL_THRESHOLD;

  // consider the "logical" state of walls AND actual range reading
  if (sense_left_wall && mouse->isWallInDirection(left_of_dir(mouse->getDir()))) { // wall is on left
//    print("wall left\n.");
    *offset = dToWallLeft + AbstractMaze::HALF_WALL_THICKNESS;
    *yaw = dir_to_yaw(mouse->getDir()) + currentYaw_l;
  } else if (sense_right_wall && mouse->isWallInDirection(right_of_dir(mouse->getDir()))) { // wall is on right
//    print("wall right\n.");
    *offset = AbstractMaze::UNIT_DIST - dToWallRight - AbstractMaze::HALF_WALL_THICKNESS;
    *yaw = dir_to_yaw(mouse->getDir()) + currentYaw_r;
  } else { // we're too far from any walls, use our pose estimation
//    print("no walls\n.");
    *offset = WallFollower::dispToLeftEdge(mouse);
    *yaw = mouse->getPose().yaw;
  }

  return out;
};

double WallFollower::dispToNextEdge(Mouse *mouse) {
  Pose current_pose = mouse->getPose();
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
    default:
      return std::numeric_limits<double>::quiet_NaN();
  }
}

double WallFollower::fwdDispToCenter(Mouse *mouse) {
  Pose current_pose = mouse->getPose();
  double next_row_y = (mouse->getRow() + 1) * AbstractMaze::UNIT_DIST;
  double next_col_x = (mouse->getCol() + 1) * AbstractMaze::UNIT_DIST;
  double row_offset = next_row_y - current_pose.y;
  double col_offset = next_col_x - current_pose.x;

  double row_offset_to_center = row_offset - AbstractMaze::HALF_UNIT_DIST;
  double col_offset_to_center = col_offset - AbstractMaze::HALF_UNIT_DIST;
  Direction dir = mouse->getDir();

  switch (dir) {
    case Direction::S:
      return row_offset_to_center;
    case Direction::N:
      return -row_offset_to_center;
    case Direction::E:
      return col_offset_to_center;
    case Direction::W:
      return -col_offset_to_center;
    default:
      return std::numeric_limits<double>::quiet_NaN();
  }
}

double WallFollower::forwardDisplacement(Direction dir, Pose start_pose, Pose end_pose) {
  switch (dir) {
    case Direction::N:
      return start_pose.y - end_pose.y;
    case Direction::E:
      return end_pose.x - start_pose.x;
    case Direction::S:
      return end_pose.y - start_pose.y;
    case Direction::W:
      return start_pose.x - end_pose.x;
    default:
      return std::numeric_limits<double>::quiet_NaN();
  }
}

double WallFollower::dispToLeftEdge(Mouse *mouse) {
  double row_offset = mouse->getRowOffsetToEdge();
  double col_offset = mouse->getColOffsetToEdge();
  Direction dir = mouse->getDir();

  switch (dir) {
    case Direction::S:
      return AbstractMaze::UNIT_DIST - col_offset;
    case Direction::N:
      return col_offset;
    case Direction::W:
      return AbstractMaze::UNIT_DIST - row_offset;
    case Direction::E:
      return row_offset;
    default:
      return std::numeric_limits<double>::quiet_NaN();
  }
}

double WallFollower::yawDiff(double y1, double y2) {
  double diff = y2 - y1;
  if (diff > M_PI) return diff - M_PI * 2;
  if (diff < -M_PI) return diff + M_PI * 2;
  return diff;
}

