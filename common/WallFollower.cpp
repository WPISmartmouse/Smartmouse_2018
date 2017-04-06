#include "WallFollower.h"
#include "RobotConfig.h"

const double WallFollower::kPWall = 1.0;
const double WallFollower::kDWall = 50;
const double WallFollower::kPYaw = 1.4;

WallFollower::WallFollower(RobotConfig config) : disp(0.0), goalDisp(AbstractMaze::UNIT_DIST), dispError(goalDisp),
                                                 config(config), lastLeftWallError(0), lastRightWallError(0) {}

WallFollower::WallFollower(double goalDisp) : disp(0.0), goalDisp(goalDisp), dispError(goalDisp), lastLeftWallError(0),
                                              lastRightWallError(0) {}

std::pair<double, double>
WallFollower::compute_wheel_velocities(Mouse *mouse, Pose start_pose, RangeData range_data) {
  Pose current_pose = mouse->getPose();
  disp = forwardDisplacement(mouse->getDir(), start_pose, current_pose);

  double currentYaw = current_pose.yaw;
  angleError = yawDiff(dir_to_yaw(mouse->getDir()), currentYaw);
  dToWallLeft = sin(config.ANALOG_ANGLE + angleError) * range_data.left_analog
                + sin(angleError) * config.SIDE_ANALOG_X
                + cos(angleError) * config.SIDE_ANALOG_Y;
  dToWallRight = -(sin(-config.ANALOG_ANGLE + angleError) * range_data.right_analog
                   + sin(angleError) * config.SIDE_ANALOG_X
                   + cos(angleError) * -config.SIDE_ANALOG_Y);


  dispError = goalDisp - disp;

  // Add corrections based on distance to walls
  double leftWallError = AbstractMaze::HALF_INNER_UNIT_DIST - dToWallLeft;
  double rightWallError = AbstractMaze::HALF_INNER_UNIT_DIST - dToWallRight;
  double dLeftWallError = leftWallError - lastLeftWallError;
  double dRightWallError = rightWallError - lastRightWallError;
  lastLeftWallError = leftWallError;
  lastRightWallError = rightWallError;

  double errorToCenter;
  std::string following;

  if (mouse->isWallInDirection(left_of_dir(mouse->getDir()))) { // wall is on left
    errorToCenter = -leftWallError;
    following = "Left wall";
  } else if (mouse->isWallInDirection(right_of_dir(mouse->getDir()))) { // wall is on right
    errorToCenter = rightWallError;
    following = "Right wall";
  } else { // we're too far from any walls, use our pose estimation
    errorToCenter = sidewayDispToCenter(mouse);
    following = "Nothing";
  }

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
  }
}

double WallFollower::sidewayDispToCenter(Mouse *mouse) {
  double row_offset_to_center = AbstractMaze::HALF_UNIT_DIST - mouse->getRowOffsetToEdge();
  double col_offset_to_center = AbstractMaze::HALF_UNIT_DIST - mouse->getColOffsetToEdge();
  Direction dir = mouse->getDir();

  switch (dir) {
    case Direction::S:
      return -col_offset_to_center;
    case Direction::N:
      return col_offset_to_center;
    case Direction::W:
      return -row_offset_to_center;
    case Direction::E:
      return row_offset_to_center;
  }
}

double WallFollower::yawDiff(double y1, double y2) {
  double diff = y2 - y1;
  if (diff > M_PI) return diff - M_PI * 2;
  if (diff < -M_PI) return diff + M_PI * 2;
  return diff;
}

