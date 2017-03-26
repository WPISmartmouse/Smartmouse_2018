#include <Arduino.h>

void setup() {
  pinMode(13, OUTPUT);
  pinMode(3, INPUT_PULLUP);
  Serial.begin(9600);
}

void loop() {
  digitalWrite(13, HIGH);
  delay(100);
  digitalWrite(13, LOW);
  delay(100);
  Serial.println("Hello world");
}
