#include <real/commands/Backup.h>

Backup::Backup() : Command("Backup"), mouse(RealMouse::inst()) {}

void Backup::initialize() {
  setTimeout(300);
  digitalWrite(RealMouse::LED_6, 1);
}

void Backup::execute() {
  mouse->setSpeedCps(-0.75, -0.75);
}

bool Backup::isFinished() {
  return isTimedOut();
}

void Backup::end() {
  digitalWrite(RealMouse::LED_6, 0);
}

