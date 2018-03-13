/**
 * Measure the impulse response of the motor, from which we can fit a model.
 */

#include <Arduino.h>
#include <real/RealMouse.h>

RealMouse *mouse;

void setup() {
  mouse = RealMouse::inst();
  mouse->setup();
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

  unsigned long now_ms = millis();
  constexpr int num_sec = 1;
  constexpr int pulse_ms = 100;
  for (unsigned long i = 0; i < 1000; ++i) {
    double t = 2 * M_PI * i / 1000.0;
    auto m1 = static_cast<int>(sin(t) * 255);
    auto m2 = static_cast<int>(sin(t) * 255);
    if (m1 > 0) {
      analogWrite(RealMouse::MOTOR_LEFT_A,m1);
      analogWrite(RealMouse::MOTOR_LEFT_B, 0);
    } else {
      analogWrite(RealMouse::MOTOR_LEFT_A, 0);
      analogWrite(RealMouse::MOTOR_LEFT_B, -m1);
    }
    if (m2 > 0) {
      analogWrite(RealMouse::MOTOR_RIGHT_A,m2);
      analogWrite(RealMouse::MOTOR_RIGHT_B, 0);
    } else {
      analogWrite(RealMouse::MOTOR_RIGHT_A, 0);
      analogWrite(RealMouse::MOTOR_RIGHT_B, -m2);
    }

    int32_t l_enc = mouse->left_encoder.read();
    int32_t r_enc = mouse->right_encoder.read();
    print("%u,%i,%i\r\n", micros(), l_enc, r_enc);
    delayMicroseconds(10);
  }

  analogWrite(RealMouse::MOTOR_LEFT_A, 0);
  analogWrite(RealMouse::MOTOR_LEFT_B, 0);
  analogWrite(RealMouse::MOTOR_RIGHT_A, 0);
  analogWrite(RealMouse::MOTOR_RIGHT_B, 0);

  delay(10 * 1000);
}