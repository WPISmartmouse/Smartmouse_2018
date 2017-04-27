#include <tuple>
#include "ForwardToDiagonal.h"

ForwardToDiagonal::ForwardToDiagonal() : Command("FwdToDiagonal"), mouse(RealMouse::inst()) {}

void ForwardToDiagonal::initialize() {
  digitalWrite(RealMouse::LED_4, 1);
  start = mouse->getGlobalPose();
  driver.start(start, DriveStraight::fwdDispToDiag(mouse));
}

void ForwardToDiagonal::execute() {
  double l_adjust, r_adjust;
  std::tie(l_adjust, r_adjust) = driver.compute_wheel_velocities(this->mouse);
  l_adjust = config.MAX_SPEED - l_adjust;
  r_adjust = config.MAX_SPEED - r_adjust;
  double l = driver.dispError * kDisp - l_adjust;
  double r = driver.dispError * kDisp - r_adjust;
  mouse->setSpeed(l, r);
}

bool ForwardToDiagonal::isFinished() {
  return fabs(driver.dispError) <= 0.003;
}

void ForwardToDiagonal::end() {
  digitalWrite(RealMouse::LED_4, 0);
}
