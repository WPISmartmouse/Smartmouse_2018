#include "WaitForStart.h"
#include "RealMouse.h"

WaitForStart::WaitForStart() : CommandGroup("wait") {
}

bool WaitForStart::isFinished() {
  return !digitalRead(RealMouse::inst()->BUTTON_PIN);
}
