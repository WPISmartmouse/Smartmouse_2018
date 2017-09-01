#include <common/math/math.h>
#include "DumbArcTurn.h"

const double DumbArcTurn::kP = 0.12;

DumbArcTurn::DumbArcTurn(Direction dir) : Command("SimDumbArcTurn"), mouse(SimMouse::inst()), goal_dir(dir) {}

void DumbArcTurn::initialize() {
  goalYaw = dir_to_yaw(goal_dir);
  start_dir = mouse->getDir();
}

void DumbArcTurn::execute() {
  double s;
  if (goal_dir == left_of_dir(start_dir)) {
    s = dYaw * kP;
    mouse->setSpeed(0, s);
  } else {
    s = -dYaw * kP;
    mouse->setSpeed(s, 0);
  }
}

bool DumbArcTurn::isFinished() {
  double currentYaw = mouse->getGlobalPose().yaw;
  dYaw = smartmouse::math::yawDiff(currentYaw, goalYaw);
  double vl, vr;
  std::tie(vl, vr) = mouse->getWheelVelocities();
  return fabs(dYaw) < config.ROT_TOLERANCE;
}

void DumbArcTurn::end() {
  mouse->internalTurnToFace(goal_dir);
}
