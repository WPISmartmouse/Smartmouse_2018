#include <iostream>
#include <sim/lib/SimMouse.h>
#include <common/core/Mouse.h>
#include "WaitForStart.h"

WaitForStart::WaitForStart() : Command("wait"), mouse(SimMouse::inst()) {}

void WaitForStart::initialize() {
  print("Reset mouse pose, then press enter to begin...\n");

  // grab the latest sensor readings
  auto adc = mouse->range_data_adc;

  // for each sensor, compare the ADC value to what it should be since we know we know the starting distance of each sensor

  mouse->back_left_model = smartmouse::kc::BACK_LEFT_MODEL;
  mouse->front_left_model = smartmouse::kc::FRONT_LEFT_MODEL;
  mouse->gerald_left_model = smartmouse::kc::GERALD_LEFT_MODEL;
  mouse->front_model = smartmouse::kc::FRONT_MODEL;
  mouse->gerald_right_model = smartmouse::kc::GERALD_RIGHT_MODEL;
  mouse->front_right_model = smartmouse::kc::FRONT_RIGHT_MODEL;
  mouse->back_right_model = smartmouse::kc::BACK_RIGHT_MODEL;
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

