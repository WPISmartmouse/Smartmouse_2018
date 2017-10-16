#include <real/RealMouse.h>
#include "Forward.h"

Forward::Forward() : Command("Forward"), mouse(RealMouse::inst()) {}


void Forward::initialize() {
  start = mouse->getGlobalPose();
  mouse->kinematic_controller.enable_sensor_pose_estimate = true;
  mouse->kinematic_controller.start(start, KinematicController::dispToNextEdge(mouse));
  digitalWrite(RealMouse::LED_1, 1);
}

void Forward::execute() {
  double l, r;
  std::tie(l, r) = mouse->kinematic_controller.compute_wheel_velocities(this->mouse);
  mouse->setSpeed(l, r);
}

bool Forward::isFinished() {
  return mouse->kinematic_controller.drive_straight_state.dispError <= 0;
}

void Forward::end() {
  digitalWrite(RealMouse::LED_1, 0);
  mouse->kinematic_controller.enable_sensor_pose_estimate = false;
}

