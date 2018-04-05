#include <real/RealMouse.h>
#include <real/commands/ForwardToCenter.h>

ForwardToCenter::ForwardToCenter() : Command("FwdToCenter"), mouse(RealMouse::inst()) {}

void ForwardToCenter::initialize() {
  start = mouse->getGlobalPose();
  const double goal_disp = KinematicController::fwdDispToCenter(*mouse);
  const double v0 = mouse->kinematic_controller.getCurrentForwardSpeedCUPS();
  const double vf = 0.0;
  profile = new smartmouse::kc::VelocityProfile(start, {goal_disp, v0, vf});
  digitalWrite(RealMouse::LED_3, 1);
}

void ForwardToCenter::execute() {
  double l, r;
  double t_s = static_cast<double>(getTime()) / 1000.0;
  std::tie(l, r) = profile->drive_straight_wheel_velocities(*mouse, t_s);
//  print_slow("%f, %f\r\n", l, r);
  mouse->setSpeedCps(l, r);
}

bool ForwardToCenter::isFinished() {
  return profile->dispError() <= 0;
}

void ForwardToCenter::end() {
  digitalWrite(RealMouse::LED_3, 0);
}
