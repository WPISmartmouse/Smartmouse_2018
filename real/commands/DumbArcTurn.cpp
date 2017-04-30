#include "DumbArcTurn.h"
#include<tuple>
#include <common/KinematicController/KinematicController.h>

const double DumbArcTurn::kP = 0.2;

DumbArcTurn::DumbArcTurn(Direction dir) : Command("RealDumbArcTurn"), mouse(RealMouse::inst()), goal_dir(dir) {}

void DumbArcTurn::initialize() {
  digitalWrite(RealMouse::LED_5, 1);
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

  // when we get close to aligned, there might be a wall we can use to better estimate our angle
  // this allows us to use that
  if (fabs(dYaw) < config.ROT_TOLERANCE && mouse->kinematic_controller.enable_sensor_pose_estimate) {
    mouse->kinematic_controller.enable_sensor_pose_estimate = false;
    // FIXME: this is kind of a hack. It's needed because DriveStraight checks dir in order to compute
    // FIXME: the correct yaw. it adds dir_to_yaw(getDir()), so we must assume we're close enough
    mouse->internalTurnToFace(goal_dir);
  }
}

bool DumbArcTurn::isFinished() {
  double currentYaw = mouse->getGlobalPose().yaw;
  dYaw = KinematicController::yawDiff(currentYaw, goalYaw);
  double vl, vr;
  std::tie(vl, vr) = mouse->getWheelVelocities();
  return fabs(dYaw) < config.ROT_TOLERANCE;
}

void DumbArcTurn::end() {
  mouse->internalTurnToFace(goal_dir);
  digitalWrite(RealMouse::LED_5, 0);
}
