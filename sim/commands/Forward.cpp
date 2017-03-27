#ifdef SIM

#include <SimMouse.h>
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
  start = mouse->getExactPose();
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

//  if (range_data.front_binary) {
//    disp = AbstractMaze::UNIT_DIST - SimMouse::FRONT_BINARY_THRESHOLD;
//  }
//  else {
//    disp = forwardDisplacement(start, mouse->getExactPose());
//  }
  disp = forwardDisplacement(start, mouse->getExactPose());

  double currentYaw = mouse->getExactPose().Rot().Yaw();
  double angleError = yawDiff(toYaw(mouse->getDir()), currentYaw);
  double dToWallLeft = sin(SimMouse::ANALOG_ANGLE + angleError) * range_data.left_analog
                       + sin(angleError) * SimMouse::SIDE_ANALOG_X
                       + cos(angleError) * SimMouse::SIDE_ANALOG_Y;
  double dToWallRight = -(sin(-SimMouse::ANALOG_ANGLE + angleError) * range_data.right_analog
                        + sin(angleError) * SimMouse::SIDE_ANALOG_X
                        + cos(angleError) * -SimMouse::SIDE_ANALOG_Y);

  double dispError = AbstractMaze::UNIT_DIST - disp;
  l = dispError * kPDisp;
  r = dispError * kPDisp;

  l = l > SimMouse::MAX_SPEED ? SimMouse::MAX_SPEED : l;
  r = r > SimMouse::MAX_SPEED ? SimMouse::MAX_SPEED : r;

  l = l < SimMouse::MIN_SPEED ? SimMouse::MIN_SPEED : l;
  r = r < SimMouse::MIN_SPEED ? SimMouse::MIN_SPEED : r;

  if (!checkedWalls && dispError < AbstractMaze::HALF_UNIT_DIST) {
    checkedWalls = true;
    walls[static_cast<int>(right_of_dir(mouse->getDir()))] =
            dToWallRight < SimMouse::WALL_DIST;
    walls[static_cast<int>(left_of_dir(mouse->getDir()))] =
            dToWallLeft < SimMouse::WALL_DIST;
  }

  double leftWallError = AbstractMaze::HALF_INNER_UNIT_DIST - dToWallLeft;
  double rightWallError = AbstractMaze::HALF_INNER_UNIT_DIST - dToWallRight;

  printf("%f %f\n", leftWallError, rightWallError);

  if (dToWallLeft < AbstractMaze::HALF_INNER_UNIT_DIST) {
    double correction = leftWallError * kPWall * dispError;
    r -= correction;
  } else if (dToWallRight < AbstractMaze::HALF_INNER_UNIT_DIST) {
    double correction = rightWallError * kPWall * dispError;
    l -= correction;
  }

  printf("%f %f\n", l, r);
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
