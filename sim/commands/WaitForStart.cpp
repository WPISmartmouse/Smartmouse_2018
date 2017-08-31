#include <iostream>
#include <sim/lib/SimMouse.h>
#include "WaitForStart.h"

WaitForStart::WaitForStart() : Command("wait") {}

void WaitForStart::initialize() {
  print("Reset mouse pose, then press enter to begin...\n");
}

void WaitForStart::execute() {
}

bool WaitForStart::isFinished() {
  return std::cin.get();
}

void WaitForStart::end() {
//  SimMouse::inst()->resetToStartPose();
}

