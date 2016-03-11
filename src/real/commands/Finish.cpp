#ifdef EMBED
#include "Finish.h"
#include <Arduino.h>

Finish::Finish() : Command("end"){}

void Finish::initialize() {
  Serial.println("end.");
}

bool Finish::isFinished() {
  return true;
}
#endif
