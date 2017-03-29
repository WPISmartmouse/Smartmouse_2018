#pragma once

#include <Arduino.h>

#include <Encoder.h>
#include <common/Mouse.h>
#include <common/KinematicMotorController/RegulatedMotor.h>
#include <common/KinematicMotorController/KinematicController.h>


class RealMouse : public Mouse {
public:
  /** runs setup things like pin initializes */
  void setup();

  static RealMouse *inst();

  virtual SensorReading checkWalls();

private:
  RealMouse();

  static RealMouse *instance;

  const static int ENCODER1A = 0;
  const static int ENCODER1B = 0;
  const static int ENCODER2A = 0;
  const static int ENCODER2B = 0;

  const static int MOTOR1A = 0;
  const static int MOTOR1B = 0;
  const static int MOTOR2A = 0;
  const static int MOTOR2B = 0;

  const static int MOTORDIR1 = 0;
  const static int MOTORDIR2 = 0;
};
