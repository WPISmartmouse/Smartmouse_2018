#include <common/math/math.h>
#include "TurnInPlace.h"

TurnInPlace::TurnInPlace(Direction dir) : Command("SimTurnInPlace"), mouse(SimMouse::inst()), dir(dir) {}

void TurnInPlace::initialize() {
  goalYaw = dir_to_yaw(dir);
  mouse->kinematic_controller.enable_sensor_pose_estimate = false;
}

void TurnInPlace::execute() {
  double s;
  s = dYaw * kP;
  mouse->setSpeedCps(s, -s);

  // when we get close to aligned, there might be a wall we can use to better estimate our angle
  // this allows us to use that
  if (fabs(dYaw) < smartmouse::kc::ROT_TOLERANCE * 4 && mouse->kinematic_controller.enable_sensor_pose_estimate) {
    mouse->kinematic_controller.enable_sensor_pose_estimate = true;
    std::cout << "enabling sensor pose estimate\n";
    // FIXME: this is kind of a hack. It's needed because DriveStraight checks dir in order to compute
    // FIXME: the correct yaw. it adds dir_to_yaw(getDir()), so we must assume we're close enough
    mouse->internalTurnToFace(dir);
  }
}

bool TurnInPlace::isFinished() {
  double currentYaw = mouse->getGlobalPose().yaw;
  dYaw = smartmouse::math::yaw_diff(currentYaw, goalYaw);
  double vl_cps, vr_cps;
  std::tie(vl_cps, vr_cps) = mouse->getWheelVelocitiesCPS();
  return (fabs(dYaw) < smartmouse::kc::ROT_TOLERANCE) && fabs(vl_cps) <= smartmouse::kc::MIN_SPEED_CUPS
      && fabs(vr_cps) < smartmouse::kc::MIN_SPEED_CUPS;
}

void TurnInPlace::end() {
  mouse->internalTurnToFace(dir);
  mouse->kinematic_controller.enable_sensor_pose_estimate = true;
}

