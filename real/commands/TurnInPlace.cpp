#include <tuple>
#include "TurnInPlace.h"

TurnInPlace::TurnInPlace(Direction dir) : Command("RealTurnInPlace"), mouse(RealMouse::inst()), dir(dir) {}

void TurnInPlace::initialize() {
  mouse->kinematic_controller.ignore_sensor_pose_estimate = true;
  goalYaw = dir_to_yaw(dir);
  digitalWrite(RealMouse::LED_2, 1);
}

void TurnInPlace::execute() {
  double s;
  s = dYaw * kP;
  s = limit(s);
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
  if (x > 0) {
    return fmax(fmin(x, RealMouse::config.MAX_SPEED), RealMouse::config.MIN_SPEED);
  }
  else if (x < 0) {
    return fmin(fmax(x, -RealMouse::config.MAX_SPEED), -RealMouse::config.MIN_SPEED);
  }
  else return 0;
}

bool TurnInPlace::isFinished() {
  double currentYaw = mouse->getPose().yaw;
  dYaw = WallFollower::yawDiff(currentYaw, goalYaw);
  double vl, vr;
  std::tie(vl, vr) = mouse->getWheelVelocities();
  print("%f %f %f\n", vl, vr, dYaw);
  return (fabs(dYaw) < RealMouse::config.ROT_TOLERANCE) && fabs(vl) < 0.05 && fabs(vr) < 0.05;
}

void TurnInPlace::end() {
  digitalWrite(RealMouse::LED_2, 0);
  printf("done turning in place.\n");
  mouse->internalTurnToFace(dir);
}

