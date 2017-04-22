#include <common/Mouse.h>
#include "WaitForStart.h"
#include "RealMouse.h"

WaitForStart::WaitForStart() : Command("wait_calibrate"), mouse(RealMouse::inst()) {
}

void WaitForStart::initialize() {
  mouse->resetToStartPose();
}

void WaitForStart::execute() {
}

bool WaitForStart::isFinished() {
  return !digitalRead(RealMouse::BUTTON_PIN);
}

void WaitForStart::end() {
  digitalWrite(RealMouse::SYS_LED, 0);
}
