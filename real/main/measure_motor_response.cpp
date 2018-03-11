/**
 * This program is for measuring how much CPU encoder counting eats up.
 * We do this by comparing the amount of time a blocking piece of code takes when motors are moving versus stationary.
 * When the motor is stationary, the encoders will not generate any interrupts.
 * When they are moving, the encoder will generate lots of interrupts and the same for loop will take up more time.
 */

#include <Arduino.h>
#include <real/RealMouse.h>

RealMouse *mouse;

void setup() {
  mouse = RealMouse::inst();
  mouse->setup();
  mouse->kinematic_controller.enabled = false;
}

unsigned long m = 0;
void loop() {
  // turn off the motors
  analogWrite(RealMouse::MOTOR_LEFT_A, 0);
  analogWrite(RealMouse::MOTOR_LEFT_B, 0);
  analogWrite(RealMouse::MOTOR_RIGHT_A, 0);
  analogWrite(RealMouse::MOTOR_RIGHT_B, 0);
  delay(1000);

  // turn on left motor for split second
  analogWrite(RealMouse::MOTOR_LEFT_A, 0);
  analogWrite(RealMouse::MOTOR_LEFT_B, 255);

  unsigned long now_ms = millis();
  constexpr int n_sec = 10;
  constexpr int pulse_ms = 500;
  int32_t last_enc = mouse->left_encoder.read();
  while (millis() - now_ms < n_sec * 1000)  {
    // stop the motor after a brief pulse
    if (millis() - now_ms > pulse_ms) {
      analogWrite(RealMouse::MOTOR_LEFT_A, 0);
      analogWrite(RealMouse::MOTOR_LEFT_B, 0);

    }
    int32_t enc = mouse->left_encoder.read();
    double v_rps = RealMouse::tick_to_rad(enc - last_enc) / 0.001;
    print("%0.6f\n", v_rps);
    last_enc = enc;
    delay(1);
  }
}