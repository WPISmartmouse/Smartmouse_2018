#ifdef SIM

#include "WallFollower.h"

WallFollower::WallFollower() : disp(0.0), goalDisp(AbstractMaze::UNIT_DIST), dispError(goalDisp) {}
WallFollower::WallFollower(double goalDisp) : disp(0.0), goalDisp(goalDisp), dispError(goalDisp) {}

std::pair<double, double>
WallFollower::compute_wheel_velocities(SimMouse* mouse, ignition::math::Pose3d start_pose, SimMouse::RangeData range_data) {
  ignition::math::Pose3d current_pose = mouse->getExactPose();
  disp = forwardDisplacement(mouse->getDir(), start_pose, current_pose);

  double currentYaw = current_pose.Rot().Yaw();
  angleError = yawDiff(toYaw(mouse->getDir()), currentYaw);
  dToWallLeft = sin(SimMouse::ANALOG_ANGLE + angleError) * range_data.left_analog
                       + sin(angleError) * SimMouse::SIDE_ANALOG_X
                       + cos(angleError) * SimMouse::SIDE_ANALOG_Y;
  dToWallRight = -(sin(-SimMouse::ANALOG_ANGLE + angleError) * range_data.right_analog
                          + sin(angleError) * SimMouse::SIDE_ANALOG_X
                          + cos(angleError) * -SimMouse::SIDE_ANALOG_Y);

  dispError = goalDisp - disp;
  double l = FORWARD_SPEED;
  double r = FORWARD_SPEED;

  l = l > SimMouse::MAX_SPEED ? SimMouse::MAX_SPEED : l;
  r = r > SimMouse::MAX_SPEED ? SimMouse::MAX_SPEED : r;

  l = l < SimMouse::MIN_SPEED ? SimMouse::MIN_SPEED : l;
  r = r < SimMouse::MIN_SPEED ? SimMouse::MIN_SPEED : r;

  double leftWallError = fabs(AbstractMaze::HALF_INNER_UNIT_DIST - dToWallLeft);
  double rightWallError = fabs(AbstractMaze::HALF_INNER_UNIT_DIST - dToWallRight);

  if (dToWallLeft < AbstractMaze::HALF_INNER_UNIT_DIST) {
    double correction = leftWallError * kPWall;
    r -= correction;
  } else if (dToWallRight < AbstractMaze::HALF_INNER_UNIT_DIST) {
    double correction = rightWallError * kPWall;
    l -= correction;
  } else if (dToWallLeft > AbstractMaze::HALF_INNER_UNIT_DIST) {
    double correction = leftWallError * kPWall;
    l -= correction;
  } else if (dToWallRight > AbstractMaze::HALF_INNER_UNIT_DIST) {
    double correction = rightWallError * kPWall;
    r -= correction;
  }

  return std::pair<double, double>(l, r);
}

double WallFollower::forwardDisplacement(Direction dir, ignition::math::Pose3d start_pose, ignition::math::Pose3d end_pose) {
  switch (dir) {
    case Direction::N:
      return end_pose.Pos().Y() - start_pose.Pos().Y();
    case Direction::E:
      return end_pose.Pos().X() - start_pose.Pos().X();
    case Direction::S:
      return start_pose.Pos().Y() - end_pose.Pos().Y();
    case Direction::W:
      return start_pose.Pos().X() - end_pose.Pos().X();
  }
}

double WallFollower::yawDiff(double y1, double y2) {
  double diff = y2 - y1;
  if (diff > M_PI) return diff - M_PI * 2;
  if (diff < -M_PI) return diff + M_PI * 2;
  return diff;
}

double WallFollower::dispToEdge(Direction dir, ignition::math::Pose3d current_pose) {
  double x = current_pose.Pos().X() + AbstractMaze::UNIT_DIST * 8;
  double y = -current_pose.Pos().Y() + AbstractMaze::UNIT_DIST * 8;
  double row_offset = fmod(y, AbstractMaze::UNIT_DIST);
  double col_offset = fmod(x, AbstractMaze::UNIT_DIST);

  switch (dir) {
    case Direction::N:
      return AbstractMaze::UNIT_DIST - row_offset;
    case Direction::S:
      return row_offset;
    case Direction::E:
      return AbstractMaze::UNIT_DIST -  col_offset;
    case Direction::W:
      return col_offset;
  }
}
#endif
