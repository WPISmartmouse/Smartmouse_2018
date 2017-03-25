#include "Stop.h"
#include "RealMouse.h"

Stop::Stop() : Command("end") {}

bool Stop::isFinished() {
  return true;
}

void Stop::end(){
  Serial1.println("DONE.");
}
