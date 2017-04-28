#pragma once

#include <Arduino.h>
#include <Encoder.h>
#include <common/Mouse.h>
#include <common/Pose.h>
#include <common/KinematicController/KinematicController.h>


class IRConverter {
public:
  IRConverter();
  double adcToMeters(int adc);
  void calibrate(int avg_adc_value_on_center);
private:
  double calibration_offset;
  std::array<int, 18> ir_lookup;

};

class RealMouse : public Mouse {
public:
  static constexpr int TICKS_PER_REV = 900;
  static constexpr double RAD_PER_TICK = 2 * M_PI / TICKS_PER_REV;

  static const unsigned int FRONT_RIGHT_ANALOG_PIN = A18;
  static const unsigned int BACK_RIGHT_ANALOG_PIN = A14;
  static const unsigned int FRONT_ANALOG_PIN = A16;
  static const unsigned int FRONT_LEFT_ANALOG_PIN = A17;
  static const unsigned int BACK_LEFT_ANALOG_PIN = A15;
  static const unsigned int GERALD_LEFT_ANALOG_PIN = A20;
  static const unsigned int GERALD_RIGHT_ANALOG_PIN = A19;

  static const unsigned int ENCODER_LEFT_A = 9;
  static const unsigned int ENCODER_LEFT_B = 10;
  static const unsigned int ENCODER_RIGHT_A = 12;
  static const unsigned int ENCODER_RIGHT_B = 11;

  static const unsigned int MOTOR_LEFT_A = 5;
  static const unsigned int MOTOR_LEFT_B = 6;
  static const unsigned int MOTOR_RIGHT_A = 8;
  static const unsigned int MOTOR_RIGHT_B = 7;

  static const unsigned int SYS_LED = 13;
  static const unsigned int LED_1 = 25;
  static const unsigned int LED_2 = 26;
  static const unsigned int LED_3 = 27;
  static const unsigned int LED_4 = 28;
  static const unsigned int LED_5 = 29;
  static const unsigned int LED_6 = 30;
  static const unsigned int LED_7 = 31;
  static const unsigned int LED_8 = 32;

  static const unsigned int BUTTON_PIN = 23;

  static RealMouse *inst();

  static const RobotConfig config;

  virtual SensorReading checkWalls() override;

  virtual GlobalPose getGlobalPose() override;

  virtual LocalPose getLocalPose() override;

  RangeData getRangeData();

  virtual double getRowOffsetToEdge() override;

  virtual double getColOffsetToEdge() override;

  std::pair<double, double> getWheelVelocities();

  void resetToStartPose();

  void run(double dt_s);

  void setSpeed(double l_mps, double r_mps);

  /** runs setup things like pin initializes */
  void setup();

  KinematicController kinematic_controller;
  Encoder left_encoder, right_encoder;
  IRConverter ir_converter;
  double left_angle_rad;
  double right_angle_rad;

private:
  RealMouse();

  double tick_to_rad(int ticks);

  static RealMouse *instance;

  RangeData range_data;
};
