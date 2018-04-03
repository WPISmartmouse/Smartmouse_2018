#include <common/core/Mouse.h>
#include <real/commands/Calibrate.h>

Calibrate::Calibrate() : Command("calibrate"), mouse(RealMouse::inst()) {}

void Calibrate::initialize() {
  digitalWrite(RealMouse::LED_6, HIGH);
  auto adc = mouse->range_data_adc;
  mouse->back_left_model.calibrate(adc.back_left);
  mouse->front_left_model.calibrate(adc.front_left);
  mouse->gerald_left_model.calibrate(adc.gerald_left);
  mouse->front_model.calibrate(adc.front);
  mouse->gerald_right_model.calibrate(adc.gerald_right);
  mouse->front_right_model.calibrate(adc.front_right);
  mouse->back_right_model.calibrate(adc.back_right);
}

void Calibrate::execute() {}

bool Calibrate::isFinished() {
  return true;
}

void Calibrate::end() {
  digitalWrite(RealMouse::LED_6, LOW);
}
