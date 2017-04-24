#include <real/RealMouse.h>
#include <tuple>
#include "ForwardToCenter.h"

ForwardToCenter::ForwardToCenter() : Command("FwdToCenter"), mouse(RealMouse::inst()) {}

void ForwardToCenter::initialize() {
  setTimeout(2000);
  start = mouse->getPose();
  follower.goalDisp = WallFollower::fwdDispToCenter(mouse);
  digitalWrite(RealMouse::LED_3, 1);
}

void ForwardToCenter::execute() {
  range_data = mouse->getRangeData();

  double l_adjust, r_adjust;
  std::tie(l_adjust, r_adjust) = follower.compute_wheel_velocities(this->mouse, start, range_data);
  l_adjust = config.MAX_SPEED - l_adjust;
  r_adjust = config.MAX_SPEED - r_adjust;
  double l = follower.dispError * kDisp - l_adjust;
  double r = follower.dispError * kDisp - r_adjust;
  mouse->setSpeed(l, r);
}

bool ForwardToCenter::isFinished() {
  return fabs(follower.dispError) <= 0.002 || isTimedOut();
}

void ForwardToCenter::end() {
  digitalWrite(RealMouse::LED_3, 0);
}
