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

  static const int FRONT_ANALOG_PIN = A5;
  static const int FRONT_LEFT_ANALOG_PIN = A3;
  static const int BACK_LEFT_ANALOG_PIN = A4;
  static const int FRONT_RIGHT_ANALOG_PIN = A2;
  static const int BACK_RIGHT_ANALOG_PIN = A1;

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

  RangeData getRangeData();

  virtual double getRowOffsetToEdge() override;

  virtual double getColOffsetToEdge() override;

  std::pair<double, double> getWheelVelocities();

  void run(double dt_s);

  void setSpeed(double l_mps, double r_mps);

  /** runs setup things like pin initializes */
  void setup();

  KinematicMotorController kinematic_controller;

  bool ignore_sensor_pose_estimate;

private:
  RealMouse();

  double tick_to_rad(int ticks);

  static RealMouse *instance;

  static double adcToMeters(int adc);

  static const int ir_lookup[18];

  Encoder left_encoder, right_encoder;
  Pose estimated_pose;
  double row_offset_to_edge;
  double col_offset_to_edge;
};
