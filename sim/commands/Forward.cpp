#include <sim/lib/SimMouse.h>
#include "Forward.h"

Forward::Forward() : Command("Forward"), mouse(SimMouse::inst()) {}


void Forward::initialize() {
  start = mouse->getGlobalPose();
  mouse->kinematic_controller.enable_sensor_pose_estimate = true;
  mouse->kinematic_controller.start(start, KinematicController::dispToNextEdge(mouse));
}

void Forward::execute() {
  double l, r;
  std::tie(l, r) = mouse->kinematic_controller.compute_wheel_velocities(this->mouse);
  mouse->setSpeedCps(l, r);
}

bool Forward::isFinished() {
  return mouse->kinematic_controller.drive_straight_state.dispError <= 0;
}

void Forward::end() {
}

