#ifdef SIM

#include <iostream>
#include "WaitForStart.h"
#include "RealMouse.h"

WaitForStart::WaitForStart() : Command("wait") {}

void WaitForStart::initialize(){}

void WaitForStart::execute(){}

bool WaitForStart::isFinished(){
  return std::cin.get();
}

void WaitForStart::end(){}
#endif
