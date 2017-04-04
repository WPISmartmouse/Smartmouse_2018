#pragma once

#include <Arduino.h>
#include <Encoder.h>
#include <common/Mouse.h>
#include <common/Pose.h>
#include <common/KinematicMotorController/KinematicController.h>


class RealMouse : public Mouse {
public:
  const static int FRONT_BINARY_PIN = 0;
  const static int LEFT_BINARY_PIN = 0;
  const static int RIGHT_BINARY_PIN = 0;

  const static int ENCODER1A = 0;
  const static int ENCODER1B = 0;
  const static int ENCODER2A = 0;
  const static int ENCODER2B = 0;

  const static int MOTOR1A = 2;
  const static int MOTOR1B = 3;
  const static int MOTOR2A = 5;
  const static int MOTOR2B = 4;

  const static int SYS_LED = 13;
  const static int LED_1 = 33;
  const static int LED_2 = 34;
  const static int LED_3 = 35;
  const static int LED_4 = 36;
  const static int LED_5 = 37;
  const static int LED_6 = 38;
  const static int LED_7 = 39;

  static RealMouse *inst();

  virtual SensorReading checkWalls() override;

  virtual Pose getPose() override;

  virtual double getRowOffsetToEdge() override;

  virtual double getColOffsetToEdge() override;

  void run(double dt_s);

  /** runs setup things like pin initializes */
  void setup();

private:
  RealMouse();

  static RealMouse *instance;


  KinematicMotorController kinematic_controller;

  Encoder left_encoder, right_encoder;
  double row_offset_to_edge;
  double col_offset_to_edge;
};
