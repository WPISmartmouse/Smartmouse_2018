#include <real/RealMouse.h>
#include <tuple>
#include "ForwardToCenter.h"

ForwardToCenter::ForwardToCenter() : Command("FwdToCenter"), mouse(RealMouse::inst()) {}

void ForwardToCenter::initialize() {
  setTimeout(2000);
  start = mouse->getGlobalPose();
  mouse->kinematic_controller.enable_sensor_pose_estimate = true;
  driver.start(start, DriveStraight::fwdDispToCenter(mouse));
  digitalWrite(RealMouse::LED_3, 1);
}

void ForwardToCenter::execute() {
  range_data = mouse->getRangeData();

  double l_adjust, r_adjust;
  std::tie(l_adjust, r_adjust) = driver.compute_wheel_velocities(this->mouse);
  l_adjust = config.MAX_SPEED - l_adjust;
  r_adjust = config.MAX_SPEED - r_adjust;
  double l = driver.dispError * kDisp - l_adjust;
  double r = driver.dispError * kDisp - r_adjust;
  mouse->setSpeed(l, r);
}

bool ForwardToCenter::isFinished() {
  double vl, vr;
  std::tie(vl, vr) = mouse->getWheelVelocities();
  return fabs(driver.dispError) <= 0.004 || isTimedOut();
}

void ForwardToCenter::end() {
  digitalWrite(RealMouse::LED_3, 0);
  mouse->kinematic_controller.enable_sensor_pose_estimate = false;
}
