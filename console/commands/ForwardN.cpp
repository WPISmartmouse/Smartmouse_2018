#include <console/ConsoleMouse.h>
#include <iostream>
#include "ForwardN.h"

ForwardN::ForwardN(unsigned int n) : Command("ForwardN"), n(n), i(0), mouse(ConsoleMouse::inst())  {}

void ForwardN::initialize() {
}

void ForwardN::execute() {
  mouse->internalForward();
  std::cin.get();
  mouse->print_maze_mouse();
}

bool ForwardN::isFinished() {
  return ++i > n;
}

void ForwardN::end() {}

