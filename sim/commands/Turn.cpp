#ifdef SIM
#include "Turn.h"

Turn::Turn(Direction dir) : mouse(SimMouse::inst()), dir(dir),
    l(0), r(0) {}

void Turn::initialize(){
  start = mouse->getPose();
  goalYaw = toYaw(dir);
}

double Turn::yawDiff(double y1, double y2){
  double diff = y2 - y1;
  if (diff > M_PI) {
    return diff - 2*M_PI;
  }
  if (diff < -M_PI) {
    return diff + 2*M_PI;
  }
  return diff;
}

void Turn::execute(){
  l = dYaw * kP;
  r = -dYaw * kP;

  if (l < SimMouse::MIN_SPEED && l >= 0) l = SimMouse::MIN_SPEED;
  if (r < SimMouse::MIN_SPEED && r >= 0) r = SimMouse::MIN_SPEED;
  if (l > -SimMouse::MIN_SPEED && l <= 0) l = -SimMouse::MIN_SPEED;
  if (r > -SimMouse::MIN_SPEED && r <= 0) r = -SimMouse::MIN_SPEED;
  if (l > SimMouse::MAX_SPEED) l = SimMouse::MAX_SPEED;
  if (r > SimMouse::MAX_SPEED) r = SimMouse::MAX_SPEED;
  if (l < -SimMouse::MAX_SPEED) l = -SimMouse::MAX_SPEED;
  if (r < -SimMouse::MAX_SPEED) r = -SimMouse::MAX_SPEED;

  mouse->setSpeed(l,r);
}

bool Turn::isFinished(){
  double currentYaw = mouse->getPose().Rot().Yaw();
  dYaw = yawDiff(currentYaw, goalYaw);
  return (mouse->getDir() == dir) || (fabs(dYaw) < Mouse::ROT_TOLERANCE);
}

void Turn::end(){
  mouse->internalTurnToFace(dir);
  mouse->setSpeed(0, 0);
}
#endif
