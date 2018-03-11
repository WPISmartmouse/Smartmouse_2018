#include <tuple>
#include <common/math/math.h>

#include "TurnInPlace.h"

TurnInPlace::TurnInPlace(Direction dir) : Command("RealTurnInPlace"), mouse(RealMouse::inst()), dir(dir) {}

void TurnInPlace::initialize() {
  setTimeout(2000);
  goalYaw = dir_to_yaw(dir);
  digitalWrite(RealMouse::LED_2, 1);
}

void TurnInPlace::execute() {
  double s;
  s = dYaw * kP;
  mouse->setSpeedCps(-s, s);
}

bool TurnInPlace::isFinished() {
  double currentYaw = mouse->getGlobalPose().yaw;
  dYaw = smartmouse::math::yaw_diff(currentYaw, goalYaw);
  double vl, vr;
  std::tie(vl, vr) = mouse->getWheelVelocities();
  return isTimedOut() || (fabs(dYaw) < smartmouse::kc::ROT_TOLERANCE);
}

void TurnInPlace::end() {
  digitalWrite(RealMouse::LED_2, 0);
  mouse->internalTurnToFace(dir);
}

