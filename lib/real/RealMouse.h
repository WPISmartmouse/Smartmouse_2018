#pragma once

#include <Arduino.h>
#define M_PI 3.141592653589

#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Wire.h>
#include <Encoder.h>
#include <RegulatedMotor.h>
#include <VL6180X.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "KinematicController.h"
#include "Mouse.h"
#include "AbstractMaze.h"
#include "Pose.h"
#include "Bounce2.h"

class RealMouse : public Mouse {
public:
  /** runs setup things like pin initializes */
  void setup();

  constexpr static float CENTER_TO_SENSOR = 0.025; //meters
  constexpr static float HALF_SQUARE = 0.084; //meters
  constexpr static float WALL_DIST_SETPOINT = HALF_SQUARE - CENTER_TO_SENSOR;

  const static int LEDGO = 13;
  const static int LEDR = 52;
  const static int LEDG = 41;
  const static int LEDB = 51;
  const static int BUZZER = 46;

  constexpr static float WALL_DIST = 0.12;
  constexpr static float MAX_SPEED = 500; //mm/s
  constexpr static float MAX_ROT_SPEED = M_PI/2; //rad/s
  constexpr static float MIN_SPEED = 30; //mm/s
  constexpr static float MIN_ROT_SPEED = M_PI/12; //rad/s

  Bounce goButton;
  Bounce aButton;
  Bounce bButton;

  static RealMouse *inst();

  Adafruit_BNO055 imu;
  Adafruit_SSD1306 display;

  virtual SensorReading checkWalls() override;

  // \brief get pose (x, y, yaw) in (meters, meters, radians 0-2pi)
  Pose getPose();

  // \brief read IMU and inject current angle into kinematic controller
  void updateGlobalYaw();

  void setSpeed(int forwardVelocity, float ccwVelocity);

  void suggestWalls(bool *walls);

  void run();

  // \brief return array of 3 distances in meters
  float *getRawDistances();

private:
  RealMouse();

  static RealMouse *instance;

  const static int OLED_RESET = 5;
  const static int VL6180X_ADDRESS = 0x29;

  const static int IREMITTER1 = A1;
  const static int IREMITTER2 = A4;
  const static int IREMITTER3 = A6;
  const static int IREMITTER4 = A9;
  const static int IREMITTER5 = A11;
  const static int IREMITTER6 = CANRX;

  const static int IRRECEIVER1 = A2;
  const static int IRRECEIVER2 = A5;
  const static int IRRECEIVER3 = A7;
  const static int IRRECEIVER4 = A10;
  const static int IRRECEIVER5 = DAC1;
  const static int IRRECEIVER6 =
      A3; // wired to non adc pin. this does not work.

  const static int BATTERYSENSE = A0;

  const static int VL6180EN1 = 22;
  const static int VL6180EN2 = 23;
  const static int VL6180EN3 = 24;
  const static int VL6180EN4 = 25;
  const static int VL6180EN5 = 26;
  const static int VL6180EN6 = 27;

  const static int ENCODER1A = 3;
  const static int ENCODER1B = 29;
  const static int ENCODER2A = 2;
  const static int ENCODER2B = 28;

  const static int MOTOR1A = 9;  // PWML4 PC21
  const static int MOTOR1B = 11; //
  const static int MOTOR2A = 10;
  const static int MOTOR2B = 12;

  const static int MOTORDIR1 = 30;
  const static int MOTORDIR2 = 31;

  const static int SDCS = 4;
  const static int SDCARDDETECT = 42;

  const static int BUTTON1 = 48;
  const static int BUTTON2 = 50;
  const static int BUTTONGO = 44;

  VL6180X left_rangefinder;
  VL6180X right_rangefinder;
  VL6180X middle_rangefinder;

  Encoder encL;
  Encoder encR;

  Pose current_pose;

  long encLCount;
  long encRCount;

  RegulatedMotor motL;
  RegulatedMotor motR;

  KinematicController kc;

  int fwdtarget;
  int ccwtarget;

  bool walls[4];
  bool suggestedWalls[4];
  bool hasSuggestion;
  float rawDistances[3];
};
