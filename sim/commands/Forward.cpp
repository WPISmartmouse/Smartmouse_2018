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
  start = mouse->getPose();
  disp = 0.0f;
}

float Forward::yawDiff(float y1, float y2){
  float diff = y2 - y1;
  if (diff > 180) return diff - 360;
  if (diff < -180) return diff + 360;
  return diff;
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
    printf("R=%f L=%f\n", dToWallLeft, dToWallRight);
    walls[static_cast<int>(right_of_dir(mouse->getDir()))] =
      dToWallRight < SimMouse::WALL_DIST;
    walls[static_cast<int>(left_of_dir(mouse->getDir()))] =
      dToWallLeft < SimMouse::WALL_DIST;
  }

  if (isinf(distances[0]) && wallOnRight){
    wallOnRight = false;
    printf("lost right wall\n");
  }
  if (isinf(distances[2]) && wallOnLeft){
    wallOnLeft = false;
    printf("lost left wall\n");
  }

  if (wallOnRight) {
    float wallError = 0.084 - dToWallRight;
    float correction = wallError * kPWall * dispError;
    //printf("%f %f : ", wallError, correction);
    l -= correction;
  }
  else if (wallOnLeft) {
    float wallError = 0.084 - dToWallLeft;
    float correction = wallError * kPWall * dispError;
    //printf("%f %f : ", wallError, correction);
    r -= correction;
  }

  //printf("(%f,%f)\n", l, r);

  l = l > SimMouse::MAX_SPEED ? SimMouse::MAX_SPEED : l;
  r = r > SimMouse::MAX_SPEED ? SimMouse::MAX_SPEED : r;

  l = l < SimMouse::MIN_SPEED ? SimMouse::MIN_SPEED : l;
  r = r < SimMouse::MIN_SPEED ? SimMouse::MIN_SPEED : r;

  mouse->setSpeed(l,r);
}

bool Forward::isFinished(){
  return disp > AbstractMaze::UNIT_DIST;
}

void Forward::end(){
  mouse->internalForward();
  mouse->setSpeed(0,0);

  walls[static_cast<int>(mouse->getDir())] = distances[1] < SimMouse::WALL_DIST;
  walls[static_cast<int>(opposite_direction(mouse->getDir()))] = false;

  printf("predicts=%d, %d, %d, %d\n", walls[0], walls[1], walls[2], walls[3]);
  mouse->suggestWalls(walls);
}
#endif
