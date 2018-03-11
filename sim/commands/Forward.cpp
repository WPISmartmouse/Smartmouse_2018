#include <sim/lib/SimMouse.h>
#include "Forward.h"

Forward::Forward() : Command("Forward"), mouse(SimMouse::inst()) {}


void Forward::initialize() {
  start = mouse->getGlobalPose();
  mouse->kinematic_controller.enable_sensor_pose_estimate = true;
  const double goal_disp = KinematicController::dispToNextEdge(*mouse);
  const double v0 = mouse->kinematic_controller.getCurrentForwardSpeedCUPS();
  const double vf = smartmouse::kc::kVf;
  drive_straight_state  = new smartmouse::kc::DriveStraightState(start, goal_disp, v0, vf);
}

void Forward::execute() {
  double l, r;
  double t_s = static_cast<double>(getTime()) / 1000.0;
  std::tie(l, r) = drive_straight_state->compute_wheel_velocities(*mouse, t_s);
  mouse->setSpeedCps(l, r);
}

bool Forward::isFinished() {
  return drive_straight_state->dispError() <= 0.0;
}

void Forward::end() {
}

