#include "ForwardN.h"

ForwardN::ForwardN(unsigned int n) : Command("Forward"), mouse(RealMouse::inst()), n(n), profile(nullptr) {}

void ForwardN::initialize() {
  start = mouse->getGlobalPose();
  const double goal_disp = KinematicController::dispToNthEdge(*mouse, n);
  const double v0 = mouse->kinematic_controller.getCurrentForwardSpeedCUPS();
  const double vf = smartmouse::kc::kVf_cps;
  profile = new smartmouse::kc::VelocityProfile(start, {goal_disp, v0, vf});
  digitalWrite(RealMouse::LED_4, 1);
}

void ForwardN::execute() {
  double l, r;
  double t_s = static_cast<double>(getTime()) / 1000.0;
  std::tie(l, r) = profile->drive_straight_wheel_velocities(*mouse, t_s);

//  auto p = mouse->getGlobalPose();
//  print("%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %d, %d\r\n",
//        mouse->kinematic_controller.left_motor.setpoint_rps,
//        mouse->kinematic_controller.left_motor.velocity_rps,
//        mouse->kinematic_controller.right_motor.setpoint_rps,
//        mouse->kinematic_controller.right_motor.velocity_rps,
//        p.row,
//        p.col,
//        p.yaw,
//        mouse->kinematic_controller.sense_left_wall,
//        mouse->kinematic_controller.sense_right_wall);

  mouse->setSpeedCps(l, r);
}

bool ForwardN::isFinished() {
  return profile->dispError() <= 0;
}

void ForwardN::end() {
  digitalWrite(RealMouse::LED_4, 0);
}

