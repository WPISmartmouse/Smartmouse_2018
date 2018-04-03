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

  smartmouse::ir::ModelParams front_model{1.304576, 0.034583, 295.613524, 0.083000};
  smartmouse::ir::ModelParams gerald_right_model{1.187237, 0.020162, 658.131265, 0.087692};
  smartmouse::ir::ModelParams back_right_model{1.235639, 0.026170, 388.385273, 0.054205};
  smartmouse::ir::ModelParams gerald_left_model{1.195759, 0.020983, 531.855338, 0.087671};
  smartmouse::ir::ModelParams front_left_model{1.197716, 0.021642, 370.645580, 0.059513};
  smartmouse::ir::ModelParams back_left_model{1.240250, 0.028287, 380.648090, 0.054208};
  smartmouse::ir::ModelParams front_right_model{1.168262, 0.017652, 506.691267, 0.059520};

 private:
  RealMouse();

  static RealMouse *instance;
};
