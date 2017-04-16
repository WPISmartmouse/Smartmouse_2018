#include "TurnInPlace.h"

TurnInPlace::TurnInPlace(Direction dir) : Command("SimTurnInPlace"), mouse(SimMouse::inst()), dir(dir) {}

void TurnInPlace::initialize() {
  goalYaw = dir_to_yaw(dir);
  mouse->kinematic_controller.ignore_sensor_pose_estimate = true;
}

void TurnInPlace::execute() {
  double s;
  s = limit(dYaw * kP);
  mouse->setSpeed(-s, s);

  // when we get close to aligned, there might be a wall we can use to better estimate our angle
  // this allows us to use that
  if (fabs(dYaw) < 0.1 && mouse->kinematic_controller.ignore_sensor_pose_estimate) {
    mouse->kinematic_controller.ignore_sensor_pose_estimate = false;
    // FIXME: this is kind of a hack. It's needed because WallFollower checks dir in order to compute
    // FIXME: the correct yaw. it adds dir_to_yaw(getDir()), so we must assume we're close enough
    mouse->internalTurnToFace(dir);
  }
}

double TurnInPlace::limit(double x) {
  return std::fmax(std::fmin(x, SimMouse::CONFIG.MAX_SPEED), -SimMouse::CONFIG.MAX_SPEED);
}

bool TurnInPlace::isFinished() {
  double currentYaw = mouse->getPose().yaw;
  dYaw = WallFollower::yawDiff(currentYaw, goalYaw);
  double vl, vr;
  std::tie(vl, vr) = mouse->getWheelVelocities();
  return (fabs(dYaw) < SimMouse::CONFIG.ROT_TOLERANCE) && fabs(vl) < 0.05 && fabs(vr) < 0.05;
}

void TurnInPlace::end() {
  mouse->internalTurnToFace(dir);
}

