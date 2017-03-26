#include <Arduino.h>
#include "End.h"

End::End() : Command("end") {}

bool End::isFinished() {
  return true;
}

void End::end(){
  Serial.println("DONE.");
}
