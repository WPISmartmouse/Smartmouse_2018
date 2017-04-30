#include <tuple>
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
  mouse->setSpeed(-s, s);
}

bool TurnInPlace::isFinished() {
  double currentYaw = mouse->getGlobalPose().yaw;
  dYaw = KinematicController::yawDiff(currentYaw, goalYaw);
  double vl, vr;
  std::tie(vl, vr) = mouse->getWheelVelocities();
  return isTimedOut() || (fabs(dYaw) < config.ROT_TOLERANCE);
}

void TurnInPlace::end() {
  digitalWrite(RealMouse::LED_2, 0);
  mouse->internalTurnToFace(dir);
}

