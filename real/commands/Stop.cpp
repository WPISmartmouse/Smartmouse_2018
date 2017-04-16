#include <real/RealMouse.h>
#include "Stop.h"

Stop::Stop() : Command("end") {}

Stop::Stop(unsigned long stop_time) : Command("end"), stop_time(stop_time) {}

void Stop::initialize() {
  setTimeout(stop_time);
  RealMouse::inst()->setSpeed(0, 0);
}

bool Stop::isFinished() {
  return isTimedOut();
}

void Stop::end() {
}
