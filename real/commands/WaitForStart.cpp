#include <common/Mouse.h>
#include "WaitForStart.h"
#include "Calibrate.h"

bool WaitForStart::calibrated = false;

WaitForStart::WaitForStart() : CommandGroup("wait_calibrate"), mouse(RealMouse::inst()) {
  mouse->kinematic_controller.enabled = false;
  if (!calibrated) {
    addSequential(new Calibrate());
    calibrated = true;
  }
}

void WaitForStart::execute() {
  CommandGroup::execute();

  // Set the max speed of the robot based on the right wheel
  double percent_speed = fmod(mouse->right_angle_rad, TWO_PI) / TWO_PI;
  speed =  percent_speed * config.MAX_HARDWARE_SPEED;

  int idx = percent_speed * 100 / 7;
  for (int i = 0; i < 7; i++) {
    if (i < idx) {
      digitalWrite(RealMouse::LED_7 - i, 1);
    } else {
      digitalWrite(RealMouse::LED_7 - i, 0);
    }
  }

  // Set arc turn on or off based on left wheel
  config.ARC_TURN = fmod(fabs(mouse->left_angle_rad), TWO_PI) > PI;
  if (config.ARC_TURN) {
    digitalWrite(RealMouse::LED_8, 1);
  } else {
    digitalWrite(RealMouse::LED_8, 0);
  }
}

bool WaitForStart::isFinished() {
  if (CommandGroup::isFinished()) {
    return !digitalRead(RealMouse::BUTTON_PIN);
  } else {
    return false;
  }
}

void WaitForStart::end() {
  config.MAX_SPEED = max(speed, 0.1);
  for (int i = 0; i < 7; i++) {
    digitalWrite(RealMouse::LED_7 - i, 0);
  }
  mouse->kinematic_controller.enabled = true;
}
