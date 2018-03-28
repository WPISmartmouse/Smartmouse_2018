/**
 * Test the AS5048A library with the evaluation boards
 *
 * YOU MUST WIRE THIS IN THREE WIRE MODE!!!!
 */

#include <Arduino.h>

void setup() {
  analogReadResolution(13);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Serial.begin(115200);
}

void loop() {
  delayMicroseconds(1500);
  Serial.println(analogRead(A9));
}
