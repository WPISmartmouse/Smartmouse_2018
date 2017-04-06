#include "TurnInPlace.h"

TurnInPlace::TurnInPlace(Direction dir) : Command("SimTurnInPlace"), mouse(SimMouse::inst()), dir(dir) {}

void TurnInPlace::initialize() {
  goalYaw = dir_to_yaw(dir);
}

void TurnInPlace::execute() {
  double s;
  s = dYaw * kP;
  mouse->setSpeed(-s, s);
}

double TurnInPlace::limit(double x) {
  return std::fmax(std::fmin(x, SimMouse::CONFIG.MAX_SPEED), -SimMouse::CONFIG.MAX_SPEED);
}

bool TurnInPlace::isFinished() {
  double currentYaw = mouse->getPose().yaw;
  dYaw = WallFollower::yawDiff(currentYaw, goalYaw);
  double vl, vr;
  std::tie(vl, vr) = mouse->getWheelVelocities();
  return (fabs(dYaw) < Mouse::ROT_TOLERANCE) && fabs(vl) < 0.05 && fabs(vr) < 0.05;
}

void TurnInPlace::end() {
  printf("done turning in place.\n");
  mouse->internalTurnToFace(dir);
}

