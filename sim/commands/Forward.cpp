#include <sim/lib/SimMouse.h>
#include "Forward.h"

Forward::Forward() : Command("Forward"), mouse(SimMouse::inst()) {}


void Forward::initialize() {
  start = mouse->getGlobalPose();
  follower.start(start, DriveStraight::dispToNextEdge(mouse));
}

void Forward::execute() {
  double l, r;
  std::tie(l, r) = follower.compute_wheel_velocities(this->mouse);
  mouse->setSpeed(l, r);
}

bool Forward::isFinished() {
  return follower.dispError <= 0;
}

void Forward::end() {
}

