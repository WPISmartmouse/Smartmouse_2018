#include <common/Mouse.h>
#include "WaitForStart.h"
#include "RealMouse.h"

WaitForStart::WaitForStart() : Command("wait_calibrate"), mouse(RealMouse::inst()) {
}

void WaitForStart::initialize() {
}

void WaitForStart::execute() {
}

bool WaitForStart::isFinished() {
  return !digitalRead(RealMouse::BUTTON_PIN);
}
