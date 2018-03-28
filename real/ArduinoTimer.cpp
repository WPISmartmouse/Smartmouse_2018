#include "ArduinoTimer.h"
#include <Arduino.h>

unsigned long ArduinoTimer::programTimeMs() {
  return millis();
}

unsigned long ArduinoTimer::programTimeUs() {
  return micros();
}
