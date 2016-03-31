#ifdef SIM

#include "Forward.h"
#include "Direction.h"
#include "AbstractMaze.h"
#include <cmath>

Forward::Forward() : mouse(SimMouse::inst()), l(0), r(0), checkedWalls(false), wallOnLeft(true), wallOnRight(true) {}

float Forward::forwardDisplacement(ignition::math::Pose3d p0, ignition::math::Pose3d p1){
  switch(mouse->getDir()){
    case Direction::N:
      return p1.Pos().Y() - p0.Pos().Y();
      break;
    case Direction::E:
      return p1.Pos().X() - p0.Pos().X();
      break;
    case Direction::S:
      return p0.Pos().Y() - p1.Pos().Y();
      break;
    case Direction::W:
      return p0.Pos().X() - p1.Pos().X();
      break;
  }
}


void Forward::initialize(){
  mouse->resetIndicators(SimMouse::red_color);
  mouse->indicatePath(mouse->getRow(), mouse->getCol(), mouse->maze->pathToNextGoal, SimMouse::red_color);
  start = mouse->getPose();
  disp = 0.0f;
}

float Forward::yawDiff(float y1, float y2){
  float diff = y2 - y1;
  if (diff > M_PI) return diff - M_PI*2;
  if (diff < -M_PI) return diff + M_PI*2;
  return diff;
}

bool Forward::outOfRange(float range){
 return isinf(range);
}

void Forward::execute(){
  distances = mouse->getRawDistances();
  disp = forwardDisplacement(start,mouse->getPose());

  float currentYaw = mouse->getPose().Rot().Yaw();
  float angleError = yawDiff(toYaw(mouse->getDir()), currentYaw);
  float dToWallRight = distances[0] * cos(M_PI/6 + angleError);
  float dToWallLeft = distances[2] * cos(M_PI/6 - angleError);

  float dispError = AbstractMaze::UNIT_DIST - disp;
  l = dispError * kPDisp;
  r = dispError * kPDisp;

  l = l > SimMouse::MAX_SPEED ? SimMouse::MAX_SPEED : l;
  r = r > SimMouse::MAX_SPEED ? SimMouse::MAX_SPEED : r;

  l = l < SimMouse::MIN_SPEED ? SimMouse::MIN_SPEED : l;
  r = r < SimMouse::MIN_SPEED ? SimMouse::MIN_SPEED : r;

  if (!checkedWalls && dispError < AbstractMaze::UNIT_DIST/2) {
    checkedWalls = true;
    walls[static_cast<int>(right_of_dir(mouse->getDir()))] =
      dToWallRight < SimMouse::WALL_DIST;
    walls[static_cast<int>(left_of_dir(mouse->getDir()))] =
      dToWallLeft < SimMouse::WALL_DIST;
  }

  if (outOfRange(distances[0]) && wallOnRight){
    wallOnRight = false;
  }
  if (outOfRange(distances[2]) && wallOnLeft){
    wallOnLeft = false;
  }

  float rightWallError = 0.084 - dToWallRight;
  float leftWallError = 0.084 - dToWallLeft;

  if (wallOnRight) {
    float correction = rightWallError * kPWall * dispError;
    l += correction;
  }
  if (wallOnLeft) {
    float correction = leftWallError * kPWall * dispError;
    r += correction;
  }

  mouse->setSpeed(l,r);
}

bool Forward::isFinished(){
  return disp > AbstractMaze::UNIT_DIST;
}

void Forward::end(){
  mouse->resetIndicators(SimMouse::blue_color);
  mouse->indicatePath(0, 0, mouse->maze->fastest_theoretical_route, SimMouse::blue_color);

  mouse->internalForward();
  mouse->setSpeed(0,0);

  walls[static_cast<int>(mouse->getDir())] = distances[1] < SimMouse::WALL_DIST;
  walls[static_cast<int>(opposite_direction(mouse->getDir()))] = false;

  mouse->suggestWalls(walls);
}
#endif
