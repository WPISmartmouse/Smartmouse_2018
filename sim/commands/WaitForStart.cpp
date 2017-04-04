#include <stdio.h>
#include <iostream>
#include "WaitForStart.h"

WaitForStart::WaitForStart() : Command("wait") {}

void WaitForStart::initialize() {
  printf("Press enter to begin...\n");
}

void WaitForStart::execute() {
  printf("Begninning.\n");
}

bool WaitForStart::isFinished() {
  return std::cin.get();
}

void WaitForStart::end() {}

