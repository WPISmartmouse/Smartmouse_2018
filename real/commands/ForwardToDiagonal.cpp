#include <tuple>
#include "ForwardToDiagonal.h"

ForwardToDiagonal::ForwardToDiagonal() : Command("FwdToDiagonal"), mouse(RealMouse::inst()) {}

void ForwardToDiagonal::initialize() {
  digitalWrite(RealMouse::LED_4, 1);
  start = mouse->getGlobalPose();
  mouse->kinematic_controller.start(start, KinematicController::fwdDispToDiag(mouse));
}

void ForwardToDiagonal::execute() {
  double l_adjust, r_adjust;
  std::tie(l_adjust, r_adjust) = mouse->kinematic_controller.compute_wheel_velocities(this->mouse);
  l_adjust = config.MAX_SPEED - l_adjust;
  r_adjust = config.MAX_SPEED - r_adjust;
  double l = mouse->kinematic_controller.drive_straight_state.dispError * kDisp - l_adjust;
  double r = mouse->kinematic_controller.drive_straight_state.dispError * kDisp - r_adjust;
  mouse->setSpeed(l, r);
}

bool ForwardToDiagonal::isFinished() {
  return fabs(mouse->kinematic_controller.drive_straight_state.dispError) <= 0.003;
}

void ForwardToDiagonal::end() {
  digitalWrite(RealMouse::LED_4, 0);
}
