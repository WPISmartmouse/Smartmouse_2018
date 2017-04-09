#include <SimMouse.h>
#include <sim/SimMouse.h>
#include "Forward.h"

Forward::Forward() : Command("Forward"), mouse(SimMouse::inst()), follower(SimMouse::CONFIG) {}


void Forward::initialize() {
  start = mouse->getPose();
  follower.goalDisp = WallFollower::dispToNextEdge(mouse);
  print("forward\n");
}

void Forward::execute() {
  range_data = mouse->getRangeData();

  double l, r;
  std::tie(l, r) = follower.compute_wheel_velocities(this->mouse, start, range_data);
  mouse->setSpeed(l, r);

  // TODO: reset distance based on front sensor?
}

bool Forward::isFinished() {
  return follower.dispError <= 0;
}

void Forward::end() {
}

