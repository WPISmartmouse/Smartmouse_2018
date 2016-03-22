#include "WaitForStart.h"
#include "RealMouse.h"

WaitForStart::WaitForStart() : Command("wait"), down(1), up(0) {}

void WaitForStart::initialize(){}

void WaitForStart::execute(){
  Serial.println(digitalRead(RealMouse::BUTTONGO));
  if (down <=5 && !digitalRead(RealMouse::BUTTONGO)){
    down++;
  }
  if (down > 5){
    if (digitalRead(RealMouse::BUTTONGO)){
      up++;
    }
  }
}

bool WaitForStart::isFinished(){
  return down > 5 && up > 5;
}

void WaitForStart::end(){
  Serial.println("starting.");
}
