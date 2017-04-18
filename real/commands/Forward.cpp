#include <real/RealMouse.h>
#include <tuple>
#include <common/Mouse.h>
#include "Forward.h"

Forward::Forward() : Command("Forward"), mouse(RealMouse::inst()) {}


void Forward::initialize() {
  start = mouse->getPose();
  follower.goalDisp = WallFollower::dispToNextEdge(mouse);
  digitalWrite(RealMouse::LED_1, 1);
}

void Forward::execute() {
  range_data = mouse->getRangeData();

  if (range_data.front_analog < 0.08) {
    digitalWrite(RealMouse::LED_6, 1);
  }
  else {
    digitalWrite(RealMouse::LED_6, 0);
  }

  double l, r;
  std::tie(l, r) = follower.compute_wheel_velocities(this->mouse, start, range_data);
  mouse->setSpeed(l, r);
}

bool Forward::isFinished() {
  return follower.dispError <= 0;
}

void Forward::end() {
  auto p = mouse->getPose();
  print("done: %f, %f, %f\n", p.x, p.y, p.yaw);
  digitalWrite(RealMouse::LED_1, 0);
}

