#include "Calibrate.h"

Calibrate::Calibrate() : Command("calibrate") {}

bool Calibrate::isFinished() {
  return true;
}
