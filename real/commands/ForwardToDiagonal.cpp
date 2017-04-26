#include <tuple>
#include "ForwardToDiagonal.h"

ForwardToDiagonal::ForwardToDiagonal() : Command("FwdToDiagonal"), mouse(RealMouse::inst()) {}

int c = 0;

void ForwardToDiagonal::initialize() {
  start = mouse->getGlobalPose();
  driver.start(start, DriveStraight::fwdDispToDiag(mouse) - 0.05);
  c = 0;
}

void ForwardToDiagonal::execute() {
  print("exec %i\r\n", c);
  c++;
  double l_adjust, r_adjust;
  std::tie(l_adjust, r_adjust) = driver.compute_wheel_velocities(this->mouse);
  l_adjust = config.MAX_SPEED - l_adjust;
  r_adjust = config.MAX_SPEED - r_adjust;
  double l = driver.dispError * kDisp + 0.2 - l_adjust;
  double r = driver.dispError * kDisp + 0.2 - r_adjust;
  mouse->setSpeed(l, r);
}

bool ForwardToDiagonal::isFinished() {
  return driver.dispError <= 0;
}

void ForwardToDiagonal::end() {
}
