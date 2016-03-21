#include "Forward.h"

Forward::Forward() : mouse(RealMouse::inst()), l(0), r(0), checkedWalls(false), wallOnLeft(true), wallOnRight(true) {}

void Forward::initialize(){
  start = mouse->getPose();
  disp = 0.0f;
}

void Forward::execute(){
  distances = mouse->getRawDistances();
  disp = forwardDisplacement(start,mouse->getPose());

  float currentYaw = mouse->getPose().yaw;
  float angleError = yawDiff(toYaw(mouse->getDir()), currentYaw);
  float dToWallRight = distances[0] * cos(M_PI/6 + angleError);
  float dToWallLeft = distances[2] * cos(M_PI/6 - angleError);

  float dispError = AbstractMaze::UNIT_DIST - disp;
  l = dispError * kPDisp;
  r = dispError * kPDisp;

  l = l > RealMouse::MAX_SPEED ? RealMouse::MAX_SPEED : l;
  r = r > RealMouse::MAX_SPEED ? RealMouse::MAX_SPEED : r;

  l = l < RealMouse::MIN_SPEED ? RealMouse::MIN_SPEED : l;
  r = r < RealMouse::MIN_SPEED ? RealMouse::MIN_SPEED : r;

  if (!checkedWalls && dispError < AbstractMaze::UNIT_DIST/2) {
    checkedWalls = true;
    walls[static_cast<int>(right_of_dir(mouse->getDir()))] =
      dToWallRight < RealMouse::WALL_DIST;
    walls[static_cast<int>(left_of_dir(mouse->getDir()))] =
      dToWallLeft < RealMouse::WALL_DIST;
  }

  if (isinf(distances[0]) && wallOnRight){
    wallOnRight = false;
  }
  if (isinf(distances[2]) && wallOnLeft){
    wallOnLeft = false;
  }

  float rightWallError = 0.084 - dToWallRight;
  float leftWallError = 0.084 - dToWallLeft;

  if (wallOnRight) {
    float correction = rightWallError * kPWall * dispError;
    //printf("%f %f : ", wallError, correction);
    l -= correction;
  }
  if (wallOnLeft) {
    float correction = leftWallError * kPWall * dispError;
    //printf("%f %f : ", wallError, correction);
    r -= correction;
  }

  //printf("(%f,%f)\n", l, r);
  mouse->setSpeed(l,r);
}

bool Forward::isFinished(){
  return disp > AbstractMaze::UNIT_DIST;
}

void Forward::end(){
  mouse->internalForward();
  mouse->setSpeed(0,0);

  walls[static_cast<int>(mouse->getDir())] = distances[1] < RealMouse::WALL_DIST;
  walls[static_cast<int>(opposite_direction(mouse->getDir()))] = false;

  mouse->suggestWalls(walls);
}

