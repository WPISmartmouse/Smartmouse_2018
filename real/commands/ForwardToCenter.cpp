#include <real/RealMouse.h>
#include <tuple>
#include "ForwardToCenter.h"

ForwardToCenter::ForwardToCenter() : Command("FwdToCenter"), mouse(RealMouse::inst()), follower(RealMouse::CONFIG) {}

void ForwardToCenter::initialize() {
  start = mouse->getPose();
  follower.goalDisp = WallFollower::fwdDispToCenter(mouse);
  digitalWrite(RealMouse::LED_3, 1);
}

void ForwardToCenter::execute() {
  range_data = mouse->getRangeData();

  double l, r;
  std::tie(l, r) = follower.compute_wheel_velocities(this->mouse, start, range_data);
  l -= follower.disp * kDisp;
  r -= follower.disp * kDisp;
  mouse->setSpeed(l, r);
}

bool ForwardToCenter::isFinished() {
  return follower.dispError <= 0;
}

void ForwardToCenter::end() {
  digitalWrite(RealMouse::LED_3, 0);
}
