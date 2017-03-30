#ifdef SIM

#include "TurnInPlace.h"

TurnInPlace::TurnInPlace(Direction dir) : Command("SimTurn"), mouse(SimMouse::inst()), dir(dir) {}

void TurnInPlace::initialize() {
  goalYaw = toYaw(dir);
}

double TurnInPlace::yawDiff(double y1, double y2) {
  double diff = y2 - y1;
  if (diff > M_PI) {
    return diff - 2 * M_PI;
  }
  if (diff < -M_PI) {
    return diff + 2 * M_PI;
  }
  return diff;
}

void TurnInPlace::execute() {
  double l, r;
  l = -dYaw * kP;
  r = dYaw * kP;
  setSpeedLimited(l, r);
}

void TurnInPlace::setSpeedLimited(double l, double r) {
  if (l > 0) {
    l = std::fmin(l, SimMouse::MAX_SPEED);
    l = std::fmax(l, SimMouse::MIN_SPEED);
  } else {
    l = std::fmax(l, -SimMouse::MAX_SPEED);
    l = std::fmin(l, -SimMouse::MIN_SPEED);
  }

  if (r > 0) {
    r = std::fmin(r, SimMouse::MAX_SPEED);
    r = std::fmax(r, SimMouse::MIN_SPEED);
  } else {
    r = std::fmax(r, -SimMouse::MAX_SPEED);
    r = std::fmin(r, -SimMouse::MIN_SPEED);
  }

  mouse->setSpeed(l, r);
}

bool TurnInPlace::isFinished() {
  double currentYaw = mouse->getExactPose().yaw;
  dYaw = yawDiff(currentYaw, goalYaw);
  return fabs(dYaw) < Mouse::ROT_TOLERANCE;
}

void TurnInPlace::end() {
  mouse->internalTurnToFace(dir);
}

#endif
