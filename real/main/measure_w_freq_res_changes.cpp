/**
 * Measure the impulse response of the motor, from which we can fit a model.
 */

#include <Arduino.h>
#include <real/RealMouse.h>
#include <cmath>

const float STARTING_POINTS[] = {0.25f, 0.5f, 0.75f};
const int STARTING_RES = 8;
//The nth index represents the frequency of the nth resolution. 0 represents an invalid reso.
const float IDEAL_FREQS[] = {0, 0, 15000000, 7500000, 3750000, 1875000, 937500, 468750, 234375, 117187.5f, 58593.75f, 29296.875f, 14648.437f, 7324.219f, 3662.109f, 1831.055f, 915.527f};
const float DEFAULT_FREQ = 488.28;
const int HIGHEST_SUPPORTED_RES = 16;

RealMouse *mouse;

void setAllMotors(int value) {
  analogWrite(RealMouse::MOTOR_LEFT_A1, value);
  analogWrite(RealMouse::MOTOR_LEFT_A2, 0);
  analogWrite(RealMouse::MOTOR_RIGHT_B1, value);
  analogWrite(RealMouse::MOTOR_RIGHT_B2, 0);
}

void setup() {
  mouse = RealMouse::inst();
  mouse->setup();
  analogWriteFrequency(RealMouse::MOTOR_LEFT_A1, 2600);
  analogWriteFrequency(RealMouse::MOTOR_LEFT_A2, 2600);
}

unsigned long m = 0;

void loop() {
  RealMouse::checkVoltage();

  // turn off the motors
  setAllMotors(0);
  delay(2000);

  print("resolution,frequency,analog write value,change in left encoder,changein right encoder\r\n");
  for (unsigned int res = STARTING_RES; res <= HIGHEST_SUPPORTED_RES; res++) {
    analogWriteRes(res);
    for (float point : STARTING_POINTS) {
      double value = std::pow(2, res) * point;
      for (unsigned int freqMultiplier = 1; freqMultiplier < 5; freqMultiplier++) {
        float freq = DEFAULT_FREQ * freqMultiplier;
        print("%d,%f,%f,", res, freq, value);
        analogWriteFrequency(RealMouse::MOTOR_LEFT_A1, freq);
        analogWriteFrequency(RealMouse::MOTOR_LEFT_A2, freq);
        analogWriteFrequency(RealMouse::MOTOR_RIGHT_B1, freq);
        analogWriteFrequency(RealMouse::MOTOR_RIGHT_B2, freq);
        int l_enc_initial = mouse->left_encoder.getRotation();
        int r_enc_initial = mouse->right_encoder.getRotation();
        setAllMotors(static_cast<int>(value));
        delay(500);
        int l_enc_mid = mouse->left_encoder.getRotation();
        int r_enc_mid = mouse->right_encoder.getRotation();
        setAllMotors(static_cast<int>(value + 1));
        delay(500);
        int dl_enc = (mouse->left_encoder.getRotation() - l_enc_mid) - (l_enc_mid - l_enc_initial);
        int dr_enc = (mouse->right_encoder.getRotation() - r_enc_mid) - (r_enc_mid - r_enc_initial);
        print("%d,%d\r\n", dl_enc, dr_enc);
      }
    }
  }

  setAllMotors(0);
  delay(10 * 1000);
}
