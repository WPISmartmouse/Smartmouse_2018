#include <iostream>
#include <sim/lib/SimMouse.h>
#include <common/core/Mouse.h>
#include "WaitForStart.h"

bool WaitForStart::calibrated = false;

WaitForStart::WaitForStart() : Command("wait"), mouse(SimMouse::inst()) {}

void WaitForStart::initialize() {
  if (!calibrated) {

    calibrated = true;

    // grab the latest sensor readings
    auto adc = mouse->range_data_adc;

    // for each sensor, compare the ADC value to what it should be since we know we know the starting distance of each sensor
    mouse->back_left_model.calibrate(adc.back_left);
    mouse->front_left_model.calibrate(adc.front_left);
    mouse->gerald_left_model.calibrate(adc.gerald_left);
    mouse->front_model.calibrate(adc.front);
    mouse->gerald_right_model.calibrate(adc.gerald_right);
    mouse->front_right_model.calibrate(adc.front_right);
    mouse->back_right_model.calibrate(adc.back_right);
  }

  print("Reset mouse pose, then press enter to begin...\n");
}

void WaitForStart::execute() {
}

bool WaitForStart::isFinished() {
  return true;
//  return static_cast<bool>(std::cin.get());
}

void WaitForStart::end() {
//  SimMouse::inst()->resetToStartPose();
}

