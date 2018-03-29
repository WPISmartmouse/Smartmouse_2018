/**
 * Test the AS5048A library with the evaluation boards
 *
 * YOU MUST WIRE THIS IN THREE WIRE MODE!!!!
 */

#include <Arduino.h>
#include <RealMouse.h>

void setup() {
  analogReadResolution(13);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Serial.begin(115200);
}

void loop() {
  delay(2);
  print("%04i, %04i, %04i, %04i, %04i, %04i, %04i\r\n",
        analogRead(RealMouse::BACK_LEFT_ANALOG_PIN),
        analogRead(RealMouse::FRONT_LEFT_ANALOG_PIN),
        analogRead(RealMouse::GERALD_LEFT_ANALOG_PIN),
        analogRead(RealMouse::FRONT_ANALOG_PIN),
        analogRead(RealMouse::GERALD_RIGHT_ANALOG_PIN),
        analogRead(RealMouse::FRONT_RIGHT_ANALOG_PIN),
        analogRead(RealMouse::BACK_RIGHT_ANALOG_PIN));
}
