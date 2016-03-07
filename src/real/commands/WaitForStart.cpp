#include "WaitForStart.h"
#include "RealMouse.h"
/*#include <Arduino.h>*/

WaitForStart::WaitForStart() : Command("wait") {}

void WaitForStart::initialize(){}

void WaitForStart::execute(){}

bool WaitForStart::isFinished(){
  return !digitalRead(RealMouse::startButtonPin);
}

void WaitForStart::end(){}
