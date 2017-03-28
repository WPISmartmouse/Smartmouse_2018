#include "Calibrate.h"

Calibrate::Calibrate() : Command("calibrate"), mouse(RealMouse::inst()), lastDisplayUpdate(0) {}

void Calibrate::initialize() {
  mouse->display.setTextSize(1);
  mouse->display.setTextColor(WHITE);
}

void Calibrate::execute() {
  uint8_t system = 0, gyro = 0, accel = 0, mag = 0;
  mouse->imu.getCalibration(&system, &gyro, &accel, &mag);
  digitalWrite(RealMouse::LEDG, (system == 3));
  digitalWrite(RealMouse::LEDB, (mag == 3));

  uint32_t now = millis();
  if (now - lastDisplayUpdate > REFRESH_TIME) {
    lastDisplayUpdate = now;

    imu::Vector<3> euler = mouse->imu.getVector(Adafruit_BNO055::VECTOR_EULER);

    float *dist = mouse->getRawDistances();

    mouse->display.clearDisplay();
    mouse->display.setCursor(0, 0);
    mouse->display.println("S G A M");
    mouse->display.print(system);
    mouse->display.print(" ");
    mouse->display.print(gyro);
    mouse->display.print(" ");
    mouse->display.print(accel);
    mouse->display.print(" ");
    mouse->display.println(mag);

    mouse->display.println("-");
    mouse->display.print("Yaw ");
    mouse->display.println(euler.x());

    mouse->display.print("Voltage ");
    mouse->display.println(mouse->getVoltage());

    mouse->display.println("Dist L F R");
    mouse->display.print(dist[2], 3);
    mouse->display.print(" ");
    mouse->display.print(dist[1], 3);
    mouse->display.print(" ");
    mouse->display.print(dist[0], 3);
    mouse->display.display();
  }
}

bool Calibrate::isFinished() {
  return mouse->goButton.fell();
}

void Calibrate::end() {
  digitalWrite(RealMouse::LEDG, 0);
  digitalWrite(RealMouse::LEDB, 0);
}
