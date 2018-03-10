#include <real/RealMouse.h>
#include "ForwardToCenter.h"

ForwardToCenter::ForwardToCenter() : Command("FwdToCenter"), mouse(RealMouse::inst()) {}

void ForwardToCenter::initialize() {
//  setTimeout(2000);
  start = mouse->getGlobalPose();
  mouse->kinematic_controller.enable_sensor_pose_estimate = true;
  mouse->kinematic_controller.start(start, KinematicController::fwdDispToCenter(mouse), 0.0);
  digitalWrite(RealMouse::LED_3, 1);
}

void ForwardToCenter::execute() {
  double l, r;
  std::tie(l, r) = mouse->kinematic_controller.compute_wheel_velocities(this->mouse);
  mouse->setSpeedCps(l, r);
}

bool ForwardToCenter::isFinished() {
  double vl, vr;
  std::tie(vl, vr) = mouse->getWheelVelocities();
  return fabs(mouse->kinematic_controller.drive_straight_state.disp_error) <= 0.01
      or (fabs(vl) < 0.01 and fabs(vr) < 0.01);
}

void ForwardToCenter::end() {
  digitalWrite(RealMouse::LED_3, 0);
  mouse->kinematic_controller.enable_sensor_pose_estimate = false;
}
