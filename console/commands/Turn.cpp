#ifdef CONSOLE

#include "Turn.h"
#include "ConsoleMouse.h"

Turn::Turn(Direction dir) : mouse(ConsoleMouse::inst()), dir(dir) {}

void Turn::initialize() {
  mouse->internalTurnToFace(dir);
  if (!mouse->inBounds()) {
    //this is probably the most serious error possible
    //it means you've run into a wall. Just give up.
    printf("RAN INTO A WALL\n");
  }
}

void Turn::execute() {}

bool Turn::isFinished() {
  return true;
}

void Turn::end() {}

#endif
