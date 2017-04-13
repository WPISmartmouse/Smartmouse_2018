#include <real/RealMouse.h>
#include <tuple>
#include "Forward.h"

Forward::Forward() : Command("Forward"), mouse(RealMouse::inst()), follower(RealMouse::CONFIG) {}


void Forward::initialize() {
  start = mouse->getPose();
  follower.goalDisp = WallFollower::dispToNextEdge(mouse);
  digitalWrite(RealMouse::LED_4, 1);
  delay(300);
  digitalWrite(RealMouse::LED_4, 0);
  delay(300);

  digitalWrite(RealMouse::LED_1, 1);
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
  digitalWrite(RealMouse::LED_1, 0);
}

