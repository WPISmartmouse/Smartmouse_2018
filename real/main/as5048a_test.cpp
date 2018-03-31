/**
 * Test the AS5048A library with the evaluation boards
 *
 * YOU MUST WIRE THIS IN THREE WIRE MODE!!!!
 */

#include <Arduino.h>
#include <AS5048A.h>

AS5048A left(9);
AS5048A right(10);

void setup() {
  left.init();
  right.init();
  Serial.begin(0);
  pinMode(MOSI, OUTPUT);
  digitalWrite(MOSI, HIGH);
}

void loop() {
  word l = left.getRawRotation();
  word r = right.getRawRotation();
  print("%d, %d\r\n", l, r);
}
