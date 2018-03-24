/**
 * Test the AS5048A library with the evaluation boards
 *
 * YOU MUST WIRE THIS IN THREE WIRE MODE!!!!
 */

#include <Arduino.h>
#include <AS5048A.h>

AS5048A angleSensor(10);

void setup() {
  angleSensor.init();
  Serial.begin(0);
}

void loop() {
  word val = angleSensor.getRawRotation();
  Serial.print("Rotation of: 0x");
  Serial.println(val, HEX);
}
