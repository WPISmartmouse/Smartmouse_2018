#pragma once

#define LIBRARY_VERSION  2.0.0

#include <Encoder.h>

class RegulatedMotor {
public:

  RegulatedMotor(int encoderPinA, int encoderPinB, int fwdPin, int revPin);

  enum class MotorState {
    COAST, RAW_PWM, BRAKE, VELOCITY
  };

  void setup();

  bool run();

  void runNow(unsigned long deltaTime);

  void setSpeed(int speed);

  void setState(MotorState state);

  void setPID(float kp, float ki, float kd, float kvff);

  void setSampleTime(unsigned long sampleTime);

  void goPWM(int pwm);

  long getEncoder();

private:
  long thisPosition;
  long lastPosition;

  int calculatedSpeed;
  int lastCalculatedSpeed;
  int lastOutput;
  int iTerm;

  Encoder encoder;

  int fwdPin;
  int revPin;
  int encoderPinA;
  int encoderPinB;

  MotorState state;
  MotorState lastState;

  unsigned long lastTime;

  int targetSpeed;

  float kp;
  float ki;
  float kd;
  float kvff;

  unsigned long sampleTime; //MILLISECONDS
  int error;

};
