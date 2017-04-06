#include <common/Mouse.h>
#include "AlignYaw.h"

AlignYaw::AlignYaw() : Command("SimAlignYaw"), mouse(SimMouse::inst()) {}

void AlignYaw::initialize() {
}

void AlignYaw::execute() {
  range_data = mouse->getRangeData();
  double s = (range_data.left_analog - range_data.right_analog) * kP;
  printf("%f %f %f\n", range_data.left_analog, range_data.right_analog, s);
  mouse->setSpeed(-s, s);
}

double AlignYaw::limit(double x) {
  return std::fmax(std::fmin(x, SimMouse::CONFIG.MAX_SPEED), -SimMouse::CONFIG.MAX_SPEED);
}

bool AlignYaw::isFinished() {
  double vl, vr;
  std::tie(vl, vr) = mouse->getWheelVelocities();
  range_data = mouse->getRangeData();
  return fabs(range_data.left_analog - range_data.right_analog) < 0.001 && fabs(vl) < 0.05 && fabs(vr) < 0.05;
}

void AlignYaw::end() {
  printf("realgined.\n");
}

