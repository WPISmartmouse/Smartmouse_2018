#ifdef SIM

#include <SimMouse.h>
#include "Forward.h"

Forward::Forward() : mouse(SimMouse::inst()) {}


void Forward::initialize() {
  start = mouse->getEstimatedPose();
  follower.goalDisp = WallFollower::dispToEdge(mouse);
}

void Forward::execute() {
  range_data = mouse->getRangeData();

  double l, r;
  std::tie(l, r) = follower.compute_wheel_velocities(this->mouse, start, range_data);

  mouse->setSpeed(l, r);
}

bool Forward::isFinished() {
  return follower.dispError <= 0;
}

void Forward::end() {
}

#endif
