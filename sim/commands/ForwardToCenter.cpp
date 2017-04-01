#ifdef SIM

#include <SimMouse.h>
#include "ForwardToCenter.h"

ForwardToCenter::ForwardToCenter() : mouse(SimMouse::inst()) {}

void ForwardToCenter::initialize() {
  start = mouse->getEstimatedPose();
  follower.goalDisp = WallFollower::dispToCenter(mouse);
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
  printf("%f\n", follower.dispError);
  return follower.dispError <= 0;
}

void ForwardToCenter::end() {
}

#endif
