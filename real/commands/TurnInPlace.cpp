#include <tuple>
#include "TurnInPlace.h"

TurnInPlace::TurnInPlace(Direction dir) : Command("RealTurnInPlace"), mouse(RealMouse::inst()), dir(dir) {}

void TurnInPlace::initialize() {
  mouse->kinematic_controller.ignore_sensor_pose_estimate = true;
  setTimeout(2000);
  goalYaw = dir_to_yaw(dir);
  digitalWrite(RealMouse::LED_2, 1);
}

void TurnInPlace::execute() {
  double s;
  s = dYaw * kP;
//  print("SPEED: %f\n", s);
  mouse->setSpeed(-s, s);

  // when we get close to aligned, there might be a wall we can use to better estimate our angle
  // this allows us to use that
//  if (fabs(dYaw) < config.ROT_TOLERANCE * 4 && mouse->kinematic_controller.ignore_sensor_pose_estimate) {
//    mouse->kinematic_controller.ignore_sensor_pose_estimate = false;
//     FIXME: this is kind of a hack. It's needed because DriveStraight checks dir in order to compute
//     FIXME: the correct yaw. it adds dir_to_yaw(getDir()), so we must assume we're close enough
//    mouse->internalTurnToFace(dir);
//  }
}

bool TurnInPlace::isFinished() {
  double currentYaw = mouse->getGlobalPose().yaw;
  dYaw = KinematicController::yawDiff(currentYaw, goalYaw);
  double vl, vr;
  std::tie(vl, vr) = mouse->getWheelVelocities();
  return isTimedOut() || (fabs(dYaw) < config.ROT_TOLERANCE);
}

void TurnInPlace::end() {
  mouse->kinematic_controller.ignore_sensor_pose_estimate = false;
  digitalWrite(RealMouse::LED_2, 0);
  mouse->internalTurnToFace(dir);
}

