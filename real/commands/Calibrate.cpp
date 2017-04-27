#include <common/Mouse.h>
#include "Calibrate.h"

Calibrate::Calibrate() : Command("calibrate"), mouse(RealMouse::inst()) {}

void Calibrate::initialize() {
  int avg_right_adc_ticks = (analogRead(RealMouse::FRONT_RIGHT_ANALOG_PIN) + analogRead(RealMouse::BACK_RIGHT_ANALOG_PIN))/2;
  mouse->ir_converter.calibrate(avg_right_adc_ticks);
}

void Calibrate::execute() {}

bool Calibrate::isFinished() {
  return true;
}

void Calibrate::end() {}
