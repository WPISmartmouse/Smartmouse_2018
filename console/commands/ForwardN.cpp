#include <console/ConsoleMouse.h>
#include "ForwardN.h"

ForwardN::ForwardN(unsigned int n) : Command("ForwardN"), n(n), mouse(ConsoleMouse::inst())  {}

void ForwardN::initialize() {
  for (unsigned int i =0; i < n ;i++) {
    mouse->internalForward();
  }
}

void ForwardN::execute() {
}

bool ForwardN::isFinished() {
  return true;
}

void ForwardN::end() {}

