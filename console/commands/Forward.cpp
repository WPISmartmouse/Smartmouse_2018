#include "Forward.h"
#include "ConsoleMouse.h"

Forward::Forward() : mouse(ConsoleMouse::inst()) {}

void Forward::initialize() {
  mouse->internalForward();
}

void Forward::execute() {}

bool Forward::isFinished() {
  return true;
}

void Forward::end() {}

