#include "Turn.h"
#include <stdio.h>
#include <math.h>

Turn::Turn(Direction dir) : mouse(RealMouse::inst()), dir(dir), useIMU(1) {}

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
  if (mouse->getIMUCalibration() < 3) {
    useIMU = 0;
  }
}

void Turn::execute(){
  float speed = dYaw * kP + copysignf(1.0, dYaw) * minimalSpeed;

  if (speed < RealMouse::MIN_ROT_SPEED && speed >= 0) speed = RealMouse::MIN_ROT_SPEED;
  if (speed > -RealMouse::MIN_ROT_SPEED && speed <= 0) speed = -RealMouse::MIN_ROT_SPEED;
  if (speed > RealMouse::MAX_ROT_SPEED) speed = RealMouse::MAX_ROT_SPEED;
  if (speed < -RealMouse::MAX_ROT_SPEED) speed = -RealMouse::MAX_ROT_SPEED;

  mouse->setSpeed(0,speed);
}

bool Turn::isFinished(){
  float currentYaw = -1;
  if (useIMU) {
    currentYaw = mouse->getIMUYaw();
  } else {
    Pose currentPose = mouse->getPose();
    currentYaw = currentPose.yaw;
  }
  dYaw = yawDiff(currentYaw, goalYaw);
  // Serial1.print(" currentYaw");
  // Serial1.print(currentYaw);
  // Serial1.print(" goalYaw");
  // Serial1.print(goalYaw);
  // Serial1.print(" dYaw");
  // Serial1.println(dYaw);
  return (mouse->getDir() == dir) || (fabs(dYaw) < Mouse::ROT_TOLERANCE);
}

void Turn::end(){
  mouse->internalTurnToFace(dir);
  mouse->setSpeed(0,0);
  if (useIMU) {
    mouse->updateGlobalYaw();
    // trust the IMU anyways
  }
}
