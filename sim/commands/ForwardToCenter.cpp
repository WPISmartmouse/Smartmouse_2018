#include <SimMouse.h>
#include "ForwardToCenter.h"

ForwardToCenter::ForwardToCenter() : Command("FwdToCenter"), mouse(SimMouse::inst()) {}

void ForwardToCenter::initialize() {
  start = mouse->getPose();
  follower.goalDisp = WallFollower::fwdDispToCenter(mouse);
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
  return follower.dispError <= 0;
}

void ForwardToCenter::end() {
}
