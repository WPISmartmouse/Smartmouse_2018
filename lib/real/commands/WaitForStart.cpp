#include "WaitForStart.h"
#include "RealMouse.h"

WaitForStart::WaitForStart() : Command("wait"), mouse(RealMouse::inst()) {}

void WaitForStart::initialize(){
  digitalWrite(RealMouse::LEDGO,1);
}

void WaitForStart::execute(){
}

bool WaitForStart::isFinished(){
  return mouse->goButton.fell();
}

void WaitForStart::end(){
  digitalWrite(RealMouse::LEDGO,0);
  Serial.println("starting.");
}
