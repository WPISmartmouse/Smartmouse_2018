#include <real/commands/Forward.h>

Forward::Forward() : Command("Forward"), mouse(RealMouse::inst()), profile(nullptr) {}

void Forward::initialize() {
  setTimeout(3000);
  start = mouse->getGlobalPose();
  const double goal_disp = KinematicController::dispToNextEdge(*mouse);
  const double v0 = mouse->kinematic_controller.getCurrentForwardSpeedCUPS();
  const double vf = smartmouse::kc::kVf_cps;
  profile = new smartmouse::kc::VelocityProfile(start, {goal_disp, v0, vf});
  digitalWrite(RealMouse::LED_1, 1);
}

void Forward::execute() {
  double l, r;
  double t_s = static_cast<double>(getTime()) / 1000.0;
  std::tie(l, r) = profile->drive_straight_wheel_velocities(*mouse, t_s);
  mouse->setSpeedCps(l, r);
}

bool Forward::isFinished() {
  return profile->dispError() <= 0 or isTimedOut();
}

void Forward::end() {
  digitalWrite(RealMouse::LED_1, 0);
}

