#pragma once

#include <Arduino.h>
#include <Encoder.h>
#include <common/Mouse.h>
#include <common/Pose.h>
#include <common/KinematicMotorController/KinematicController.h>
#include <common/RobotConfig.h>


class RealMouse : public Mouse {
public:
  static constexpr int TICKS_PER_REV = 900;
  static constexpr double RAD_PER_TICK = 2 * M_PI / TICKS_PER_REV;

  static const int FRONT_BINARY_PIN = 0;
  static const int LEFT_BINARY_PIN = 0;
  static const int RIGHT_BINARY_PIN = 0;

  static const int ENCODER1A = 7;
  static const int ENCODER1B = 8;
  static const int ENCODER2A = 10;
  static const int ENCODER2B = 9;

  static const int MOTOR1A = 5;
  static const int MOTOR1B = 4;
  static const int MOTOR2A = 3;
  static const int MOTOR2B = 2;

  static const int SYS_LED = 13;
  static const int LED_1 = 33;
  static const int LED_2 = 34;
  static const int LED_3 = 35;
  static const int LED_4 = 36;
  static const int LED_5 = 37;
  static const int LED_6 = 38;
  static const int LED_7 = 39;

  static RealMouse *inst();
  static const RobotConfig CONFIG;

  virtual SensorReading checkWalls() override;

  virtual Pose getPose() override;

  virtual double getRowOffsetToEdge() override;

  virtual double getColOffsetToEdge() override;

  void run(double dt_s);

  void setSpeed(double l_mps, double r_mps);

  /** runs setup things like pin initializes */
  void setup();

  KinematicMotorController kinematic_controller;

private:
  RealMouse();

  double tick_to_rad(int ticks);

  static RealMouse *instance;

  Encoder left_encoder, right_encoder;
  double row_offset_to_edge;
  double col_offset_to_edge;
};
