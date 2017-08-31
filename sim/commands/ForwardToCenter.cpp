#include <sim/lib/SimMouse.h>
#include "ForwardToCenter.h"

ForwardToCenter::ForwardToCenter() : Command("FwdToCenter"), mouse(SimMouse::inst()) {}

void ForwardToCenter::initialize() {
  start = mouse->getGlobalPose();
  driver.start(start, DriveStraight::fwdDispToCenter(mouse));
}

void ForwardToCenter::execute() {
  double l_adjust, r_adjust;
  std::tie(l_adjust, r_adjust) = driver.compute_wheel_velocities(this->mouse);
  l_adjust = config.MAX_SPEED - l_adjust;
  r_adjust = config.MAX_SPEED - r_adjust;
  double l = driver.dispError * kDisp - l_adjust;
  double r = driver.dispError * kDisp - r_adjust;
  mouse->setSpeed(l, r);
}

bool ForwardToCenter::isFinished() {
  return driver.dispError <= 0;
}

void ForwardToCenter::end() {
}
