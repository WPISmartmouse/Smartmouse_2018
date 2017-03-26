#include "Delay.h"

Delay::Delay(int timeout) : Command("delay"), timeout(timeout) {}

void Delay::initialize() {
  setTimeout(timeout);
}

void Delay::execute() {
}

bool Delay::isFinished() {
  return isTimedOut();
}

void Delay::end() {
}
