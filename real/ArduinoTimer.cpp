#include "real/ArduinoTimer.h"
#include <Arduino.h>

unsigned long long ArduinoTimer::programTimeMs() {
  return millis();
}
