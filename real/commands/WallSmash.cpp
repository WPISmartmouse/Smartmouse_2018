#include <real/commands/WallSmash.h>

WallSmash::WallSmash() : Command("WallSmash"), mouse(RealMouse::inst()) {}

void WallSmash::initialize() {
  setTimeout(1000);
  digitalWrite(RealMouse::LED_5, 1);
}

void WallSmash::execute() {
  mouse->setSpeedCps(1, 1);
}

bool WallSmash::isFinished() {
  return isTimedOut();
}

void WallSmash::end() {
  digitalWrite(RealMouse::LED_5, 0);
}

