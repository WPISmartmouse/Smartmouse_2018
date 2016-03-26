#pragma once

#define LIBRARY_VERSION	2.0.0

#include <Encoder.h>

class RegulatedMotor {
  public:

    RegulatedMotor(int encoderPinA, int encoderPinB, int fwdPin, int revPin);

    enum class MotorState {COAST, RAW_PWM, BRAKE, VELOCITY};

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

    Encoder encoder;

    int encoderPinA;
    int encoderPinB;

    long thisPosition;
    long lastPosition;

    int calculatedSpeed;
    int lastCalculatedSpeed;
    int targetSpeed;

    int outputValue;

    float kp;
    float ki;
    float kd;
    float kvff;

    int iTerm;

    unsigned long lastTime;
    int lastOutput;

    unsigned long sampleTime; //MILLISECONDS

    MotorState state;
    MotorState lastState;

    int fwdPin;
    int revPin;

    int error;

};
