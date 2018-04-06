#include <SPI.h> // FIXME: I apparently did some horrible include shit because this is needed for it to compile
#include <real/commands/ForwardToCenter.h>

ForwardToCenter::ForwardToCenter() : Command("FwdToCenter"), mouse(RealMouse::inst()), profile(nullptr) {}

void ForwardToCenter::initialize() {
  start = mouse->getGlobalPose();
  const double goal_disp = KinematicController::fwdDispToCenter(*mouse);
//  const double goal_disp = 0.50;
  const double v0 = mouse->kinematic_controller.getCurrentForwardSpeedCUPS();
//  const double v0 = 0.5;
  const double vf = 0.0;
  profile = new smartmouse::kc::VelocityProfile(start, {goal_disp, v0, vf});
  digitalWrite(RealMouse::LED_3, 1);
}

void ForwardToCenter::execute() {
  double l, r;
  double t_s = static_cast<double>(getTime()) / 1000.0;
  std::tie(l, r) = profile->drive_straight_wheel_velocities(*mouse, t_s);

  auto disp_error = profile->dispError();
  if (l < .01 and r < .01 and 0 < disp_error and disp_error < 0.25) {
    l += kPFwd * disp_error;
    r += kPFwd * disp_error;
  }

  mouse->setSpeedCps(l, r);
}

bool ForwardToCenter::isFinished() {
  return profile->dispError() <= 0.005;
}

void ForwardToCenter::end() {
  digitalWrite(RealMouse::LED_3, 0);
}
