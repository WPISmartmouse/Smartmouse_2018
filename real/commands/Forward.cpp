#include <real/RealMouse.h>
#include <tuple>
#include <common/Mouse.h>
#include "Forward.h"

Forward::Forward() : Command("Forward"), mouse(RealMouse::inst()) {}


void Forward::initialize() {
  start = mouse->getGlobalPose();
  mouse->kinematic_controller.enable_sensor_pose_estimate = true;
  follower.start(start, DriveStraight::dispToNextEdge(mouse));
  digitalWrite(RealMouse::LED_1, 1);
}

void Forward::execute() {
  range_data = mouse->getRangeData();

  double l, r;
  std::tie(l, r) = follower.compute_wheel_velocities(this->mouse);
  mouse->setSpeed(l, r);
}

bool Forward::isFinished() {
  return follower.dispError <= 0;
}

void Forward::end() {
  digitalWrite(RealMouse::LED_1, 0);
  mouse->kinematic_controller.enable_sensor_pose_estimate = false;
}

