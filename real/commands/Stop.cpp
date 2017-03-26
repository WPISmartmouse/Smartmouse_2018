#include "Stop.h"
#include <Arduino.h>

Stop::Stop() : Command("end") {}

bool Stop::isFinished() {
  return true;
}

void Stop::end() {
  Serial.println("DONE.");
}
