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
  double s;
  double sqrt_y = dYaw > 0 ? pow(dYaw, 3/5) : -pow(-dYaw, 3/5);
  s = sqrt_y * kP;
  s = limit(s);
  mouse->setSpeed(-s, s);
}

double TurnInPlace::limit(double x) {
  if (x > 0) {
    x = std::fmin(x, SimMouse::MAX_SPEED);
  }
  else {
    x = std::fmax(x, -SimMouse::MAX_SPEED);
  }
  return x;
}

bool TurnInPlace::isFinished() {
  double currentYaw = mouse->getEstimatedPose().yaw;
  dYaw = yawDiff(currentYaw, goalYaw);
  return (fabs(dYaw) < Mouse::ROT_TOLERANCE) && mouse->isStopped();
}

void TurnInPlace::end() {
  mouse->internalTurnToFace(dir);
}

#endif
