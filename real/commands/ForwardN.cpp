#include <real/RealMouse.h>
#include "ForwardN.h"

ForwardN::ForwardN(unsigned int n) : Command("Forward"), mouse(RealMouse::inst()), n(n) {}


void ForwardN::initialize() {
  start = mouse->getGlobalPose();
  const double goal_disp = KinematicController::dispToNthEdge(*mouse, n);
  const double v0 = mouse->kinematic_controller.getCurrentForwardSpeedCUPS();
  const double vf = smartmouse::kc::kVf;
  profile = new smartmouse::kc::VelocityProfile(start, goal_disp, v0, vf);
  digitalWrite(RealMouse::LED_4, 1);
}

void ForwardN::execute() {
  double l, r;
  double t_s = static_cast<double>(getTime()) / 1000.0;
  std::tie(l, r) = profile->drive_straight_wheel_velocities(*mouse, t_s);
  mouse->setSpeedCps(l, r);
}

bool ForwardN::isFinished() {
  return profile->dispError() <= 0;
}

void ForwardN::end() {
  digitalWrite(RealMouse::LED_4, 0);
}

