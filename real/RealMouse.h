#pragma once

#include <Arduino.h>
#include <real/lib/AS5048A/AS5048A.h>

#include <common/core/Mouse.h>
#include <common/core/Pose.h>
#include <common/KinematicController/KinematicController.h>
#include <real/EEPROMModel.h>

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

  static const uint8_t FRONT_LEFT_ENABLE_PIN = 21;
  static const uint8_t GERALD_LEFT_ENABLE_PIN = 20;
  static const uint8_t GERALD_RIGHT_ENABLE_PIN = 22;
  static const uint8_t FRONT_RIGHT_ENABLE_PIN = 19;
  static const uint8_t FRONT_ENABLE_PIN = 18;
  static const uint8_t BACK_LEFT_ENABLE_PIN = 2;
  static const uint8_t BACK_RIGHT_ENABLE_PIN = 4;

  static const uint8_t BUTTON_PIN = 23;

  static RealMouse *inst();

  virtual SensorReading checkWalls() override;

  virtual GlobalPose getGlobalPose() override;

  virtual LocalPose getLocalPose() override;

  void readSensors();

  std::pair<double, double> getWheelVelocitiesCPS();

  void resetToStartPose();

  void run(double dt_s);

  void setSpeedCps(double l_mps, double r_mps);

  void Teleop();

  void calibrate();

  void loadCalibrate();

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
  smartmouse::ir::EEPROMModel back_left_model = {1.24069, 0.02827, 289.16550, 0.05621};
  smartmouse::ir::EEPROMModel front_left_model = {1.19328, 0.02176, 334.81050, 0.06385};
  smartmouse::ir::EEPROMModel gerald_left_model = {1.33752, 0.03757, 39.21917, 0.07390};
  smartmouse::ir::EEPROMModel front_model = {1.30223, 0.03677, 201.70852, 0.10000};
  smartmouse::ir::EEPROMModel gerald_right_model = {1.20272, 0.02267, 257.34429, 0.06928};
  smartmouse::ir::EEPROMModel front_right_model = {1.19368, 0.02113, 294.70709, 0.05959};
  smartmouse::ir::EEPROMModel back_right_model = {1.22678, 0.02502, 339.54924, 0.05220};


 private:
  RealMouse();

  static RealMouse *instance;
};
