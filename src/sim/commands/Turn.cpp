#ifdef SIM

#include "Turn.h"

#include <gazebo/msgs/msgs.hh>

Turn::Turn(Mouse *mouse, Direction dir) : mouse((SimMouse *)mouse), dir(dir),
    l(0), r(0) {}

void Turn::initialize(){
  std::unique_lock<std::mutex> lk(mouse->poseMutex);
  mouse->poseCond.wait(lk);
  start = mouse->pose;
  goalYaw = toYaw(dir);
}

float Turn::yawDiff(float y1, float y2){
  float diff = y2 - y1;
  if (diff > 180) return fabs(diff - 360);
  if (diff < -180) return fabs(diff + 360);
  return diff;
}

void Turn::execute(){
  l = -dYaw * kP;
  r = dYaw * kP;

  if (l < SimMouse::MIN_SPEED && l >= 0) l = SimMouse::MIN_SPEED;
  if (r < SimMouse::MIN_SPEED && r >= 0) r = SimMouse::MIN_SPEED;
  if (l > -SimMouse::MIN_SPEED && l <= 0) l = -SimMouse::MIN_SPEED;
  if (r > -SimMouse::MIN_SPEED && r <= 0) r = -SimMouse::MIN_SPEED;
  if (l > SimMouse::MAX_SPEED) l = SimMouse::MAX_SPEED;
  if (r > SimMouse::MAX_SPEED) r = SimMouse::MAX_SPEED;
  if (l < -SimMouse::MAX_SPEED) l = -SimMouse::MAX_SPEED;
  if (r < -SimMouse::MAX_SPEED) r = -SimMouse::MAX_SPEED;

  printf("%f,%f dyaw=%f\n", l, r, dYaw);

  mouse->setSpeed(l,r);
}

bool Turn::isFinished(){
  std::unique_lock<std::mutex> lk(mouse->poseMutex);
  mouse->poseCond.wait(lk);
  float currentYaw = mouse->pose.Rot().Yaw();
  dYaw = yawDiff(currentYaw, goalYaw);
  return (mouse->getDir() == dir) || (fabs(dYaw) < Mouse::ROT_TOLERANCE);
}

void Turn::end(){
  mouse->internalTurnToFace(dir);
  mouse->setSpeed(0,0);
}
#endif
