#ifdef CONSOLE

#include <iostream>
#include "WaitForStart.h"

WaitForStart::WaitForStart() : Command("wait") {}

void WaitForStart::initialize() {
  printf("Waiting. Press enter...\n");
}

void WaitForStart::execute() {}

bool WaitForStart::isFinished() {
  return std::cin.get();
}

void WaitForStart::end() {
}

#endif
