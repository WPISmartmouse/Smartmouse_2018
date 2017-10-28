#include <real/RealMouse.h>
#include "Stop.h"

Stop::Stop() : Command("end") {}

Stop::Stop(unsigned long stop_time) : Command("end"), stop_time(stop_time) {}

void Stop::initialize() {
  setTimeout(stop_time);
  RealMouse::inst()->setSpeedCps(0, 0);
  digitalWrite(RealMouse::LED_7, 1);
}

bool Stop::isFinished() {
  return isTimedOut();
}

void Stop::end() {
  digitalWrite(RealMouse::LED_7, 0);
}
