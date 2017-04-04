#ifdef SIM

#include "TurnInPlace.h"

TurnInPlace::TurnInPlace(Direction dir) : Command("SimTurnInPlace"), mouse(SimMouse::inst()), dir(dir) {}

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
  s = dYaw * kP;
  s = limit(s);
  mouse->setSpeed(-s, s);
}

double TurnInPlace::limit(double x) {
  return std::fmax(std::fmin(x, SimMouse::CONFIG.MAX_SPEED), -SimMouse::CONFIG.MAX_SPEED);
}

bool TurnInPlace::isFinished() {
  double currentYaw = mouse->getPose().yaw;
  dYaw = yawDiff(currentYaw, goalYaw);
  double vl, vr;
  std::tie(vl, vr) = mouse->getWheelVelocities();
  return (fabs(dYaw) < Mouse::ROT_TOLERANCE) && fabs(vl) < 0.05 && fabs(vr) < 0.05;
}

void TurnInPlace::end() {
  printf("done turning.");
  mouse->internalTurnToFace(dir);
}

#endif
