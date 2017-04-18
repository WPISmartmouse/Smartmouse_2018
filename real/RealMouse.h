#pragma once

#include <Arduino.h>
#include <Encoder.h>
#include <common/Mouse.h>
#include <common/Pose.h>
#include <common/KinematicController/KinematicController.h>


class RealMouse : public Mouse {
public:
  static constexpr int TICKS_PER_REV = 900;
  static constexpr double RAD_PER_TICK = 2 * M_PI / TICKS_PER_REV;

  static const unsigned int FRONT_ANALOG_PIN = A5;
  static const unsigned int FRONT_LEFT_ANALOG_PIN = A3;
  static const unsigned int BACK_LEFT_ANALOG_PIN = A4;
  static const unsigned int FRONT_RIGHT_ANALOG_PIN = A1;
  static const unsigned int BACK_RIGHT_ANALOG_PIN = A2;

  static const unsigned int ENCODER1A = 7;
  static const unsigned int ENCODER1B = 8;
  static const unsigned int ENCODER2A = 10;
  static const unsigned int ENCODER2B = 9;

  static const unsigned int MOTOR1A = 5;
  static const unsigned int MOTOR1B = 4;
  static const unsigned int MOTOR2A = 3;
  static const unsigned int MOTOR2B = 2;

  static const unsigned int SYS_LED = 13;
  static const unsigned int LED_1 = 33;
  static const unsigned int LED_2 = 34;
  static const unsigned int LED_3 = 35;
  static const unsigned int LED_4 = 36;
  static const unsigned int LED_5 = 37;
  static const unsigned int LED_6 = 38;
  static const unsigned int LED_7 = 39;

  static const unsigned int RESET_PIN = 0;
  static const unsigned int BUTTON_PIN = 0;

  static RealMouse *inst();

  static const RobotConfig config;

  virtual SensorReading checkWalls() override;

  virtual Pose getPose() override;

  RangeData getRangeData();

  virtual double getRowOffsetToEdge() override;

  virtual double getColOffsetToEdge() override;

  std::pair<double, double> getWheelVelocities();

  void run(double dt_s);

  void setSpeed(double l_mps, double r_mps);

  /** runs setup things like pin initializes */
  void setup();

  KinematicController kinematic_controller;

private:
  RealMouse();

  double tick_to_rad(int ticks);

  static RealMouse *instance;

  static double adcToMeters(int adc);

  static const int ir_lookup[18];

  Encoder left_encoder, right_encoder;
  RangeData range_data;
};
