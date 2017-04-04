#include "Forward.h"
#include "ConsoleMouse.h"

Forward::Forward() : mouse(ConsoleMouse::inst()) {}

void Forward::initialize() {
  mouse->internalForward();
  if (!mouse->inBounds()) {
    //this is probably the most serious error possible
    //it means you've run into a wall. Just give up.
    printf("RAN INTO A WALL\n");
  }
}

void Forward::execute() {}

bool Forward::isFinished() {
  return true;
}

void Forward::end() {}

