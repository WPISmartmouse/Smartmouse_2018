#include <SimMouse.h>
#include "ForwardToCenter.h"

ForwardToCenter::ForwardToCenter() : Command("FwdToCenter"), mouse(SimMouse::inst()), follower(SimMouse::CONFIG) {}

void ForwardToCenter::initialize() {
  start = mouse->getPose();
  follower.goalDisp = WallFollower::fwdDispToCenter(mouse);
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
}
