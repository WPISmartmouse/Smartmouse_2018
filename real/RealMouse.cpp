#include "RealMouse.h"

RealMouse *RealMouse::instance = nullptr;

RealMouse *RealMouse::inst() {
  if (instance == NULL) {
    instance = new RealMouse();
  }

  return instance;
}

RealMouse::RealMouse() {}

void RealMouse::setup() {
}

SensorReading RealMouse::checkWalls() {
}
