#include "Turn.h"
#include <stdio.h>

Turn::Turn(Direction dir) : mouse(RealMouse::inst()), dir(dir), l(0), r(0) {}

float Turn::yawDiff(float y1, float y2){
  float diff = y2 - y1;
  if (diff > M_PI) {
    return diff - 2*M_PI;
  }
  if (diff < -M_PI) {
    return diff + 2*M_PI;
  }
  return diff;
}

void Turn::initialize(){
  start = mouse->getPose();
  goalYaw = toYaw(dir);
}

void Turn::execute(){
  l = -dYaw * kP;
  r = dYaw * kP;

  if (l < RealMouse::MIN_SPEED && l >= 0) l = RealMouse::MIN_SPEED;
  if (r < RealMouse::MIN_SPEED && r >= 0) r = RealMouse::MIN_SPEED;
  if (l > -RealMouse::MIN_SPEED && l <= 0) l = -RealMouse::MIN_SPEED;
  if (r > -RealMouse::MIN_SPEED && r <= 0) r = -RealMouse::MIN_SPEED;
  if (l > RealMouse::MAX_SPEED) l = RealMouse::MAX_SPEED;
  if (r > RealMouse::MAX_SPEED) r = RealMouse::MAX_SPEED;
  if (l < -RealMouse::MAX_SPEED) l = -RealMouse::MAX_SPEED;
  if (r < -RealMouse::MAX_SPEED) r = -RealMouse::MAX_SPEED;

  mouse->setSpeed(l,r);
}

bool Turn::isFinished(){
  float currentYaw = mouse->getPose().yaw;
  dYaw = yawDiff(currentYaw, goalYaw);
  return (mouse->getDir() == dir) || (fabs(dYaw) < Mouse::ROT_TOLERANCE);
}

void Turn::end(){
  mouse->internalTurnToFace(dir);
  mouse->setSpeed(0,0);
  mouse->updateGlobalYaw();
}

