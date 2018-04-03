#pragma once

#include <Arduino.h>
#include <real/lib/AS5048A/AS5048A.h>

#include <common/core/Mouse.h>
#include <common/core/Pose.h>
#include <common/KinematicController/KinematicController.h>


class RealMouse : public Mouse {
public:

  static constexpr double tick_to_rad(int ticks) {
    return ticks * smartmouse::kc::RAD_PER_TICK;
  }

  static const unsigned int BACK_LEFT_ANALOG_PIN = A14;
  static const unsigned int FRONT_RIGHT_ANALOG_PIN = A15;
  static const unsigned int FRONT_ANALOG_PIN = A16;
  static const unsigned int GERALD_LEFT_ANALOG_PIN = A17;
  static const unsigned int FRONT_LEFT_ANALOG_PIN = A18;
  static const unsigned int GERALD_RIGHT_ANALOG_PIN = A19;
  static const unsigned int BACK_RIGHT_ANALOG_PIN = A20;
  static const unsigned int BATTERY_ANALOG_PIN = A21;

  // TODO: remove these and use new SPI encoders
  static const unsigned int MOSI = 11;
  static const unsigned int LEFT_ENCODER_CS = 9;
  static const unsigned int RIGHT_ENCODER_CS = 10;

  static const uint8_t MOTOR_LEFT_A1 = 5;
  static const uint8_t MOTOR_LEFT_A2 = 6;
  static const uint8_t MOTOR_RIGHT_B1 = 8;
  static const uint8_t MOTOR_RIGHT_B2 = 7;

  static const uint8_t LED_1 = 25;
  static const uint8_t LED_2 = 26;
  static const uint8_t LED_3 = 27;
  static const uint8_t LED_4 = 28;
  static const uint8_t LED_5 = 29;
  static const uint8_t LED_6 = 30;
  static const uint8_t LED_7 = 31;
  static const uint8_t SYS_LED = 32;

  static const uint8_t BUTTON_PIN = 23;

  static RealMouse *inst();

  virtual SensorReading checkWalls() override;

  virtual GlobalPose getGlobalPose() override;

  virtual LocalPose getLocalPose() override;

  std::pair<double, double> getWheelVelocities();

  void resetToStartPose();

  void run(double dt_s);

  void setSpeedCps(double l_mps, double r_mps);

  void calibrate();

  static double checkVoltage();


  /** runs setup things like pin initializes */
  void setup();

  KinematicController kinematic_controller;
  AS5048A left_encoder, right_encoder;
  double left_angle_rad;
  double right_angle_rad;
  RangeData<int> range_data_adc;

  /** store this as meters interally. the RoboSimState msg will be in ADC values **/
  RangeData<double> range_data_m;

  /** generated with ./compute_calibration_parameters.py right.csv left.csv front.csv **/
  smartmouse::ir::ModelParams gerald_right_model = {1.25991, 0.02697, 1164.77724, 0.06928};
  smartmouse::ir::ModelParams back_right_model = {1.25643, 0.02669, 517.55987, 0.05220};
  smartmouse::ir::ModelParams front_left_model = {1.19678, 0.02072, 768.14887, 0.06385};
  smartmouse::ir::ModelParams front_right_model = {1.23762, 0.02446, 653.84782, 0.05959};
  smartmouse::ir::ModelParams front_model = {1.27046, 0.02867, 600.86490, 0.08000};
  smartmouse::ir::ModelParams back_left_model = {1.21944, 0.02408, 577.09229, 0.05621};
  smartmouse::ir::ModelParams gerald_left_model = {1.24543, 0.02568, 654.73110, 0.07390};

 private:
  RealMouse();

  static RealMouse *instance;
};
