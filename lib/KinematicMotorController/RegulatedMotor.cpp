#include "Arduino.h"
#include <RegulatedMotor.h>
#include "../Encoder/Encoder.h"

RegulatedMotor::RegulatedMotor(long* encoder, int fwdPin, int revPin) :
  encoder(encoder), fwdPin(fwdPin), revPin(revPin), state(MOTORSTATE_COAST) { }

void RegulatedMotor::setPID(float kp, float ki, float kd, float kvff)
{
	this->kp = kp;
	this->ki = ki;
	this->kd = kd;
	this->kvff = kvff;
}

void RegulatedMotor::setSampleTime(unsigned long sampleTime)
{
	this->sampleTime = sampleTime;
  speedScale = 1000000L / sampleTime;
}

void RegulatedMotor::setSpeed(int speed){
	state = MOTORSTATE_SPEED;
	targetSpeed = speed;
}

bool RegulatedMotor::run(){
  if (state == MOTORSTATE_PWM){
    return true;
  }

	if (state == MOTORSTATE_COAST){
		goPWM(0);
		lastState = MOTORSTATE_COAST;
		return true;
	}

	if (state == MOTORSTATE_BRAKE) {
    digitalWrite(fwdPin,255);
    digitalWrite(revPin,255);
    lastState = MOTORSTATE_BRAKE;
    return true;
	}

	const int outMax = 255;
	const int outMin = -255;
	unsigned long thisTime = micros();
	unsigned long deltaTime = thisTime - lastTime;
  if(deltaTime>=sampleTime){
    thisPosition = *encoder;

    if (lastState == MOTORSTATE_COAST || lastState == MOTORSTATE_BRAKE){
    	calculatedSpeed = 0;
    	iTerm = 0;
    } else {
    	calculatedSpeed = ((thisPosition - lastPosition) * 1000000L) / deltaTime;
    }

    error = targetSpeed - calculatedSpeed;
    iTerm += (ki * error);

    if (iTerm > outMax) {
      iTerm = outMax;
    }
    else if (iTerm < outMin) {
      iTerm = outMin;
    }

    int dInput = (kd * (calculatedSpeed - lastCalculatedSpeed)) / deltaTime;

    int output = constrain(kp * error + iTerm - dInput + kvff * (long)targetSpeed,outMin,outMax);

	  goPWM(output);
    //Serial.println(error);

    lastCalculatedSpeed = calculatedSpeed;
    lastPosition = thisPosition;
    lastTime = thisTime;
    lastState = MOTORSTATE_SPEED;
	  return true;
   }
   else return false;


}

void RegulatedMotor::goPWM(int pwm){
  int apwm = constrain(abs(pwm),0,255);
  if (pwm >= 0){
    analogWrite(revPin,0);
    analogWrite(fwdPin,apwm);
    return;
  }
  if (pwm < 0){
    analogWrite(fwdPin,0);
    analogWrite(revPin,apwm);
    return;
  }
}

void RegulatedMotor::setState(int state){
	this->state = state;
}

long RegulatedMotor::getEncoder(){
  return *encoder;
}

int RegulatedMotor::getError(){
  return error;
}
