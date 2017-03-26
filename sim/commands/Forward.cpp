#ifdef SIM

#include "Forward.h"

Forward::Forward() : mouse(SimMouse::inst()), l(0), r(0), checkedWalls(false), wallOnLeft(true), wallOnRight(true) {}

double Forward::forwardDisplacement(ignition::math::Pose3d p0, ignition::math::Pose3d p1) {
  switch (mouse->getDir()) {
    case Direction::N:
      return p1.Pos().Y() - p0.Pos().Y();
    case Direction::E:
      return p1.Pos().X() - p0.Pos().X();
    case Direction::S:
      return p0.Pos().Y() - p1.Pos().Y();
    case Direction::W:
      return p0.Pos().X() - p1.Pos().X();
  }
}


void Forward::initialize() {
  mouse->resetIndicators(SimMouse::red_color);
  mouse->indicatePath(mouse->getRow(), mouse->getCol(), mouse->maze->pathToNextGoal, SimMouse::red_color);
  start = mouse->getPose();
  disp = 0.0f;
}

double Forward::yawDiff(double y1, double y2) {
  double diff = y2 - y1;
  if (diff > M_PI) return diff - M_PI * 2;
  if (diff < -M_PI) return diff + M_PI * 2;
  return diff;
}

void Forward::execute() {
  range_data = mouse->getRangeData();
  disp = forwardDisplacement(start, mouse->getPose());

  double currentYaw = mouse->getPose().Rot().Yaw();
  double angleError = yawDiff(toYaw(mouse->getDir()), currentYaw);
  double dToWallRight = range_data.right_analog * cos(M_PI / 6 + angleError);
  double dToWallLeft = range_data.left_analog * cos(M_PI / 6 - angleError);

  double dispError = AbstractMaze::UNIT_DIST - disp;
  l = dispError * kPDisp;
  r = dispError * kPDisp;

  l = l > SimMouse::MAX_SPEED ? SimMouse::MAX_SPEED : l;
  r = r > SimMouse::MAX_SPEED ? SimMouse::MAX_SPEED : r;

  l = l < SimMouse::MIN_SPEED ? SimMouse::MIN_SPEED : l;
  r = r < SimMouse::MIN_SPEED ? SimMouse::MIN_SPEED : r;

  if (!checkedWalls && dispError < AbstractMaze::UNIT_DIST / 2) {
    checkedWalls = true;
    walls[static_cast<int>(right_of_dir(mouse->getDir()))] =
            dToWallRight < SimMouse::WALL_DIST;
    walls[static_cast<int>(left_of_dir(mouse->getDir()))] =
            dToWallLeft < SimMouse::WALL_DIST;
  }

  if (!range_data.right_binary && wallOnRight) {
    wallOnRight = false;
  }
  if (!range_data.left_binary && wallOnLeft) {
    wallOnLeft = false;
  }

  double rightWallError = 0.084 - dToWallRight;
  double leftWallError = 0.084 - dToWallLeft;

  if (wallOnRight) {
    double correction = rightWallError * kPWall * dispError;
    l += correction;
  }
  if (wallOnLeft) {
    double correction = leftWallError * kPWall * dispError;
    r += correction;
  }

  mouse->setSpeed(l, r);
}

bool Forward::isFinished() {
  return disp > AbstractMaze::UNIT_DIST;
}

void Forward::end() {
  mouse->resetIndicators(SimMouse::blue_color);
  mouse->indicatePath(0, 0, mouse->maze->fastest_theoretical_route, SimMouse::blue_color);

  mouse->internalForward();
  mouse->setSpeed(0, 0);

  walls[static_cast<int>(mouse->getDir())] = range_data.front_binary;
  walls[static_cast<int>(opposite_direction(mouse->getDir()))] = false;

  mouse->suggestWalls(walls);
}

#endif
