#include "MeasureMazeOrientation.h"

MeasureMazeOrientation::MeasureMazeOrientation() :  Command("MeasureMazeOrientation"), mouse(RealMouse::inst()), MAX_SAMPLE_COUNT(20), data(0), readyToExit(0) {}

void MeasureMazeOrientation::initialize(){
  mouse->display.setTextSize(1);
  mouse->display.setTextColor(WHITE);
  mouse->display.clearDisplay();
  mouse->display.setCursor(0,0);
  mouse->display.println("Maze Cal");
  mouse->display.println("Face east [go]");
  mouse->display.display();
}

void MeasureMazeOrientation::execute(){
  uint8_t system = 0, gyro = 0, accel = 0, mag = 0;
  mouse->imu.getCalibration(&system, &gyro, &accel, &mag);
  digitalWrite(RealMouse::LEDG, (system > 0));
  if (mouse->goButton.fell()){
    digitalWrite(RealMouse::LEDR, 1);
    for (int i = 0; i < MAX_SAMPLE_COUNT; i++){
      data += mouse->getRawIMUYaw();
    }
    float eastYaw = data / MAX_SAMPLE_COUNT;
    mouse->setEastYaw(eastYaw);
    mouse->display.println(eastYaw);
    mouse->display.display();
    readyToExit = 1;
  }
}

bool MeasureMazeOrientation::isFinished(){
  return readyToExit;
}

void MeasureMazeOrientation::end(){
  digitalWrite(RealMouse::LEDG, 0);
  digitalWrite(RealMouse::LEDR, 0);
}
