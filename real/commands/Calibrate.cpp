#include <common/core/Mouse.h>
#include <real/commands/Calibrate.h>

Calibrate::Calibrate() : Command("calibrate"), mouse(RealMouse::inst()) {}

void Calibrate::initialize() {
  // TODO: implement me!
}

void Calibrate::execute() {}

bool Calibrate::isFinished() {
  return true;
}

void Calibrate::end() {}
