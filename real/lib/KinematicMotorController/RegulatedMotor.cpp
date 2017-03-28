#include <Arduino.h>
#include "RegulatedMotor.h"

RegulatedMotor::RegulatedMotor(int encoderPinA, int encoderPinB, int fwdPin, int revPin)
        : thisPosition(0),
          lastPosition(0),
          calculatedSpeed(0),
          lastCalculatedSpeed(0),
          lastOutput(0),
          iTerm(0),
          encoder(encoder),
          fwdPin(fwdPin),
          revPin(revPin),
          encoderPinA(encoderPinA),
          encoderPinB(encoderPinB),
          state(MotorState::COAST),
          lastTime(millis()) {
}

void RegulatedMotor::setup() {
  encoder.init(encoderPinA, encoderPinB);
}

void RegulatedMotor::setPID(float kp, float ki, float kd, float kvff) {
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
  this->kvff = kvff;
}

void RegulatedMotor::setSampleTime(unsigned long sampleTime) {
  this->sampleTime = sampleTime;
}

void RegulatedMotor::setSpeed(int speed) {
  state = MotorState::VELOCITY;
  targetSpeed = speed;
}

bool RegulatedMotor::run() {
  unsigned long thisTime = millis();
  unsigned long deltaTime = thisTime - lastTime;

  if (deltaTime >= sampleTime) {
    runNow(deltaTime);
    lastTime = thisTime;

    return true;
  }
  return false;
}

void RegulatedMotor::runNow(unsigned long deltaTime) {
  const int outMax = 255;
  const int outMin = -255;

  thisPosition = encoder.read();

  if (state == MotorState::RAW_PWM) {
  } else if (state == MotorState::COAST) {
    goPWM(0);
    lastState = MotorState::COAST;
  } else if (state == MotorState::BRAKE) {
    analogWrite(fwdPin, 255);
    analogWrite(revPin, 255);
    lastState = MotorState::BRAKE;
  } else if (state == MotorState::VELOCITY) {

    if (lastState == MotorState::COAST || lastState == MotorState::BRAKE) {
      calculatedSpeed = 0;
      iTerm = 0;
    } else {
      calculatedSpeed = ((thisPosition - lastPosition) * 1000L) / ((long) deltaTime);
    }

    error = targetSpeed - calculatedSpeed;
    iTerm += (ki * error);

    if (iTerm > outMax) {
      iTerm = outMax;
    } else if (iTerm < outMin) {
      iTerm = outMin;
    }

    int dTerm = (kd * (calculatedSpeed - lastCalculatedSpeed)) / ((long) deltaTime);

    int output = constrain(kp * error + iTerm - dTerm + kvff * targetSpeed, outMin, outMax);
    goPWM(output);

    lastCalculatedSpeed = calculatedSpeed;
    lastOutput = output;
    lastPosition = thisPosition;
    lastState = MotorState::VELOCITY;
  }

}

void RegulatedMotor::goPWM(int pwm) {
  int apwm = constrain(abs(pwm), 0, 255);
  if (pwm >= 0) {
    analogWrite(revPin, 0);
    analogWrite(fwdPin, apwm);
    return;
  }
  if (pwm < 0) {
    analogWrite(fwdPin, 0);
    analogWrite(revPin, apwm);
    return;
  }
}

void RegulatedMotor::setState(MotorState state) {
  this->state = state;
}

long RegulatedMotor::getEncoder() {
  return encoder.read();
}
