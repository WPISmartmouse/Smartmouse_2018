/**
 * This program is for measuring how much CPU encoder counting eats up.
 * We do this by comparing the amount of time a blocking piece of code takes when motors are moving versus stationary.
 * When the motor is stationary, the encoders will not generate any interrupts.
 * When they are moving, the encoder will generate lots of interrupts and the same for loop will take up more time.
 */

#include <real/RealMouse.h>

RealMouse *mouse;

#pragma GCC push_options
#pragma GCC optimize ("O0")
unsigned long measure() {

  // time sensitive code
  unsigned long t0 = micros();
  for (unsigned long i=0; i < 0xffffff; i++) {
  }
  unsigned long now = micros();

  return now - t0;
}
#pragma GCC pop_options

void setup() {
  mouse = RealMouse::inst();
  mouse->setup();
  mouse->kinematic_controller.enabled = false;

  print("micros per ticks\r\n");
}

unsigned long m = 0;
void loop() {
  RealMouse::checkVoltage();

  // turn off the motors
  analogWrite(RealMouse::MOTOR_LEFT_A1, 0);
  analogWrite(RealMouse::MOTOR_LEFT_A2, 0);
  analogWrite(RealMouse::MOTOR_RIGHT_B1, 0);
  analogWrite(RealMouse::MOTOR_RIGHT_B2, 0);
  delay(1000);

  unsigned long dt_while_stationary;
  dt_while_stationary = measure();

  // turn on motors
  analogWrite(RealMouse::MOTOR_LEFT_A1, 0);
  analogWrite(RealMouse::MOTOR_LEFT_A2, 255);
  analogWrite(RealMouse::MOTOR_RIGHT_B1, 0);
  analogWrite(RealMouse::MOTOR_RIGHT_B2, 255);
  delay(1000);

  // measure again
  unsigned long dt_while_moving;

  int32_t l_enc0 = mouse->left_encoder.getRotation();
  int32_t r_enc0 = mouse->right_encoder.getRotation();

  dt_while_moving = measure();

  auto d_l = (mouse->left_encoder.getRotation() - l_enc0);
  auto d_r =  (mouse->right_encoder.getRotation() - r_enc0);
  auto dticks_while_moving = d_l + d_r;

  float micros_per_tick = (float)(dt_while_moving - dt_while_stationary) / dticks_while_moving;
  print("%f\r\n", micros_per_tick);
}