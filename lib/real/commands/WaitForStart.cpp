#include "WaitForStart.h"
#include "RealMouse.h"

WaitForStart::WaitForStart() : Command("wait") {}

void WaitForStart::initialize(){}

void WaitForStart::execute(){}

bool WaitForStart::isFinished(){
  return !digitalRead(RealMouse::BUTTONGO);
}

void WaitForStart::end(){
  Serial.println("starting.");
}
