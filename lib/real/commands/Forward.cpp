#include "Forward.h"

Forward::Forward() : mouse(RealMouse::inst()), wallOnLeft(true), wallOnRight(true){}

void Forward::initialize(){
  start = mouse->getPose();
  disp = 0.0f;
}

float Forward::forwardDisplacement(Pose p0, Pose p1){
  switch(mouse->getDir()){
    case Direction::N:
      return p1.y - p0.y;
      break;
    case Direction::E:
      return p1.x - p0.x;
      break;
    case Direction::S:
      return p0.y - p1.y;
      break;
    case Direction::W:
      return p0.x - p1.x;
      break;
  }
}

float Forward::yawDiff(float y1, float y2){
  float diff = y2 - y1;
  if (diff > M_PI) return diff - M_PI*2;
  if (diff < -M_PI) return diff + M_PI*2;
  return diff;
}

bool Forward::outOfRange(float range){
  return (range >= 0.254);
}

int i=0;
void Forward::execute(){
  distances = mouse->getRawDistances();
  //TODO FIX UNITS
  disp = forwardDisplacement(start, mouse->getPose()) / 1000.0;

  float dispError = AbstractMaze::UNIT_DIST - disp;

  float currentYaw = mouse->getPose().yaw;
  float angleError = yawDiff(toYaw(mouse->getDir()), currentYaw);
  float dToWallRight = distances[0] * cos(M_PI/6 + angleError);
  float dToWallLeft = distances[2] * cos(M_PI/6 - angleError);

  float speed = dispError * kPDisp;

  float rightWallError = RealMouse::WALL_DIST_SETPOINT - dToWallRight;
  float leftWallError = RealMouse::WALL_DIST_SETPOINT - dToWallLeft;

  speed = speed > RealMouse::MAX_SPEED ? RealMouse::MAX_SPEED : speed;
  speed = speed < RealMouse::MIN_SPEED ? RealMouse::MIN_SPEED : speed;

  if (outOfRange(distances[0]) && wallOnRight){
    wallOnRight = false;
    Serial.println("lost wall on right");
  }
  if (outOfRange(distances[2]) && wallOnLeft){
    wallOnLeft = false;
    Serial.println("lost wall on left");
  }

  float correction = 0.0;
  if (wallOnLeft) {
    correction = -leftWallError * kPWall * dispError;
  }
  else if (wallOnRight) {
    correction = rightWallError * kPWall * dispError;
  }

  else if (correction > RealMouse::MAX_ROT_SPEED) {
    correction = RealMouse::MAX_ROT_SPEED;
  }
  else if (correction < -RealMouse::MAX_ROT_SPEED) {
    correction = -RealMouse::MAX_ROT_SPEED;
  }

  if ((i++)==500) {
    Serial1.print(" leftWallError:");
    Serial1.print(leftWallError);
    Serial1.print(" rightWallError:");
    Serial1.print(rightWallError);
    Serial1.print(" dispError:");
    Serial1.print(dispError);
    Serial1.print(" correction");
    Serial1.println(correction);
    i = 0;
  }

  mouse->setSpeed(speed, correction);

}

bool Forward::isFinished(){
  return disp >= AbstractMaze::UNIT_DIST;
}

void Forward::end(){
  mouse->internalForward();
  mouse->setSpeed(0,0);
}

