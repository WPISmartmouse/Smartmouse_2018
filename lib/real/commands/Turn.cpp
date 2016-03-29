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
  mouse->setSpeed(0,-M_PI/6);
}

void Turn::execute(){
}

bool Turn::isFinished(){
  Pose currentPose = mouse->getPose();
  float currentYaw = currentPose.yaw;
  dYaw = yawDiff(currentYaw, goalYaw);
  return (mouse->getDir() == dir) || (fabs(dYaw) < Mouse::ROT_TOLERANCE);
}

void Turn::end(){
  mouse->internalTurnToFace(dir);
  mouse->setSpeed(0,0);
  //mouse->updateGlobalYaw();
}

