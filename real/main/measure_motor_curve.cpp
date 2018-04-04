/**
 * Test the PID values by executing a fixed velocity profile
 */

#include <real/RealMouse.h>
#include <common/commanduino/Command.h>
#include <common/commanduino/Scheduler.h>
#include <real/ArduinoTimer.h>
#include <common/KinematicController/VelocityProfile.h>
#include <common/math/math.h>

ArduinoTimer timer;
RealMouse *mouse;
unsigned long last_t_us, last_blink_us;
bool done = false;
bool on = true;

void setup() {
  delay(1000);
  Command::setTimerImplementation(&timer);
  mouse = RealMouse::inst();
  mouse->setup();

  mouse->kinematic_controller.enabled = false;
}

void loop() {
  for (int force = 160; force < 255; force += 5) {
    RealMouse::checkVoltage();

    analogWrite(RealMouse::MOTOR_LEFT_A1, force);
    analogWrite(RealMouse::MOTOR_LEFT_A2, 0);
    analogWrite(RealMouse::MOTOR_RIGHT_B2, force);
    analogWrite(RealMouse::MOTOR_RIGHT_B1, 0);

    for (int i = 0; i < 2000; i++) {
      int tl = mouse->left_encoder.getRotation();
      int tr = mouse->right_encoder.getRotation();
      delay(1);
      if (i % 10 == 0) {
        print("%i, %i, %i\r\n", force, tl, tr);
      }
    }
  }
}
