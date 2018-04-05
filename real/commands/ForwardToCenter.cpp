#include <real/RealMouse.h>
#include <real/commands/ForwardToCenter.h>

ForwardToCenter::ForwardToCenter() : Command("FwdToCenter"), mouse(RealMouse::inst()) {}

void ForwardToCenter::initialize() {
//  setTimeout(2000);
  start = mouse->getGlobalPose();
  const double goal_disp = KinematicController::fwdDispToCenter(*mouse);
  const double v0 = mouse->kinematic_controller.getCurrentForwardSpeedCUPS();
  const double vf = 0.0;
  print("goal disp %f\r\n", goal_disp);
  profile = new smartmouse::kc::VelocityProfile(start, {goal_disp, v0, vf});
  digitalWrite(RealMouse::LED_3, 1);
}

void ForwardToCenter::execute() {
  double l, r;
  double t_s = static_cast<double>(getTime()) / 1000.0;
  std::tie(l, r) = profile->drive_straight_wheel_velocities(*mouse, t_s);
  mouse->setSpeedCps(l, r);
}

bool ForwardToCenter::isFinished() {
  double vl, vr;
  std::tie(vl, vr) = mouse->getWheelVelocities();
  return fabs(profile->dispError()) <= 0.01
      or (fabs(vl) < 0.01 and fabs(vr) < 0.01);
}

void ForwardToCenter::end() {
  digitalWrite(RealMouse::LED_3, 0);
}
