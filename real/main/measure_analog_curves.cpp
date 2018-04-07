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
  pinMode(RealMouse::BACK_LEFT_ENABLE_PIN, OUTPUT);
  pinMode(RealMouse::FRONT_LEFT_ENABLE_PIN, OUTPUT);
  pinMode(RealMouse::GERALD_LEFT_ENABLE_PIN, OUTPUT);
  pinMode(RealMouse::FRONT_ENABLE_PIN, OUTPUT);
  pinMode(RealMouse::GERALD_RIGHT_ENABLE_PIN, OUTPUT);
  pinMode(RealMouse::FRONT_RIGHT_ENABLE_PIN, OUTPUT);
  pinMode(RealMouse::BACK_RIGHT_ENABLE_PIN, OUTPUT);
  delay(5000);
}

void loop() {
  RealMouse::checkVoltage(); // THIS MUST BE IN EVERY PROGRAM

  delay(7000);

  for (int i=0; i < 10; i++) {
    delay(50);

    digitalWriteFast(RealMouse::BACK_LEFT_ENABLE_PIN, HIGH);
    digitalWriteFast(RealMouse::BACK_RIGHT_ENABLE_PIN, HIGH);
    digitalWriteFast(RealMouse::FRONT_ENABLE_PIN, HIGH);
    delayMicroseconds(100);
    auto back_left = analogRead(RealMouse::BACK_LEFT_ANALOG_PIN);
    auto back_right = analogRead(RealMouse::BACK_RIGHT_ANALOG_PIN);
    auto front = analogRead(RealMouse::FRONT_ANALOG_PIN);
    digitalWriteFast(RealMouse::BACK_RIGHT_ENABLE_PIN, LOW);
    digitalWriteFast(RealMouse::BACK_LEFT_ENABLE_PIN, LOW);
    digitalWriteFast(RealMouse::FRONT_ENABLE_PIN, LOW);

    digitalWriteFast(RealMouse::FRONT_LEFT_ENABLE_PIN, HIGH);
    digitalWriteFast(RealMouse::FRONT_RIGHT_ENABLE_PIN, HIGH);
    delayMicroseconds(100);
    auto front_left = analogRead(RealMouse::FRONT_LEFT_ANALOG_PIN);
    auto front_right = analogRead(RealMouse::FRONT_RIGHT_ANALOG_PIN);
    digitalWriteFast(RealMouse::FRONT_LEFT_ENABLE_PIN, LOW);
    digitalWriteFast(RealMouse::FRONT_RIGHT_ENABLE_PIN, LOW);

    digitalWriteFast(RealMouse::GERALD_LEFT_ENABLE_PIN, HIGH);
    digitalWriteFast(RealMouse::GERALD_RIGHT_ENABLE_PIN, HIGH);
    delayMicroseconds(100);
    auto gerald_left = analogRead(RealMouse::GERALD_LEFT_ANALOG_PIN);
    auto gerald_right = analogRead(RealMouse::GERALD_RIGHT_ANALOG_PIN);
    digitalWriteFast(RealMouse::GERALD_RIGHT_ENABLE_PIN, LOW);
    digitalWriteFast(RealMouse::GERALD_LEFT_ENABLE_PIN, LOW);

    print("%04i, %04i, %04i, %04i, %04i, %04i, %04i\r\n",
          back_left,
          front_left,
          gerald_left,
          front,
          gerald_right,
          front_right,
          back_right);
  }
}
