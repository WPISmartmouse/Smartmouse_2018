#include <sim/lib/SimMouse.h>
#include <sim/commands/Forward.h>

Forward::Forward() : Command("Forward"), mouse(SimMouse::inst()), profile(nullptr) {}

void Forward::initialize() {
  start = mouse->getGlobalPose();
  const double goal_disp = KinematicController::dispToNextEdge(*mouse);
  const double v0 = mouse->kinematic_controller.getCurrentForwardSpeedCUPS();
  const double vf = smartmouse::kc::kVf;
  profile = new smartmouse::kc::VelocityProfile(start, {goal_disp, v0, vf});
}

void Forward::execute() {
  double l, r;
  double t_s = static_cast<double>(getTime()) / 1000.0;
  std::tie(l, r) = profile->drive_straight_wheel_velocities(*mouse, t_s);
  mouse->setSpeedCps(l, r);
}

bool Forward::isFinished() {
  return profile->dispError() <= 0.0;
}

void Forward::end() {
//  mouse->pauseSim();
}

