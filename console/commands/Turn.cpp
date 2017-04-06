#include "Turn.h"
#include "ConsoleMouse.h"

Turn::Turn(Direction dir) : mouse(ConsoleMouse::inst()), dir(dir) {}

void Turn::initialize() {
  mouse->internalTurnToFace(dir);
}

void Turn::execute() {}

bool Turn::isFinished() {
  return true;
}

void Turn::end() {}

