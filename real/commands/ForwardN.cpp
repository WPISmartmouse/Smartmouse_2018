#include <real/RealMouse.h>
#include <tuple>
#include <common/Mouse.h>
#include "Forward.h"
#include "ForwardN.h"

ForwardN::ForwardN(unsigned int n) : Command("Forward"), mouse(RealMouse::inst()), n(n) {}


void ForwardN::initialize() {
  start = mouse->getGlobalPose();
  mouse->kinematic_controller.enable_sensor_pose_estimate = true;
  follower.start(start, DriveStraight::dispToNthEdge(mouse, n));
  digitalWrite(RealMouse::LED_1, 1);
}

void ForwardN::execute() {
  range_data = mouse->getRangeData();

  double l, r;
  std::tie(l, r) = follower.compute_wheel_velocities(this->mouse);
  mouse->setSpeed(l, r);
}

bool ForwardN::isFinished() {
  return follower.dispError <= 0;
}

void ForwardN::end() {
  digitalWrite(RealMouse::LED_1, 0);
  mouse->kinematic_controller.enable_sensor_pose_estimate = false;
}

