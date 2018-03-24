#include <real/RealMouse.h>
#include <real/commands/Forward.h>

Forward::Forward() : Command("Forward"), mouse(RealMouse::inst()) {}


void Forward::initialize() {
  start = mouse->getGlobalPose();
  const double goal_disp = KinematicController::dispToNextEdge(*mouse);
  const double v0 = mouse->kinematic_controller.getCurrentForwardSpeedCUPS();
  const double vf = smartmouse::kc::kVf;
  profile = new smartmouse::kc::VelocityProfile(start, {goal_disp, v0, vf});
//  mouse->kinematic_controller.start(start, KinematicController::dispToNextEdge(mouse));
  digitalWrite(RealMouse::LED_1, 1);
}

void Forward::execute() {
  double l, r;
  double t_s = static_cast<double>(getTime()) / 1000.0;
  std::tie(l, r) = profile->drive_straight_wheel_velocities(*mouse, t_s);
  mouse->setSpeedCps(l, r);
}

bool Forward::isFinished() {
  return profile->dispError() <= 0;
}

void Forward::end() {
  digitalWrite(RealMouse::LED_1, 0);
}

