#include "Arduino.h"
#include <RegulatedMotor.h>
#include "../Encoder/Encoder.h"

RegulatedMotor::RegulatedMotor(long* _encoder, int _fwdPin, int _revPin, int _pwmPin)
{
	encoder = _encoder;
	fwdPin = _fwdPin;
	revPin = _revPin;
	pwmPin = _pwmPin;
	state = MOTORSTATE_COAST;
}	

RegulatedMotor::RegulatedMotor(long* _encoder, int _fwdPin, int _revPin)
{
  encoder = _encoder;
  fwdPin = _fwdPin;
  revPin = _revPin;
  state = MOTORSTATE_COAST;
} 


void RegulatedMotor::setPID(float _kp, float _ki, float _kd, float _kvff)
{
	kp = _kp;
	ki = _ki;
	kd = _kd;
	kvff = _kvff;
}

void RegulatedMotor::setSampleTime(unsigned long _sampleTime)
{
	sampleTime = _sampleTime;
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
    	calculatedSpeed = (thisPosition - lastPosition) * speedScale;      	
    }

    error = targetSpeed - calculatedSpeed;
    iTerm += (ki * error);
    if (iTerm > outMax) iTerm= outMax;
    else if (iTerm < outMin) iTerm= outMin;
    int dInput = (calculatedSpeed - lastCalculatedSpeed);

    int output = constrain(kp * (long)error + iTerm + kd * (long)dInput + kvff * (long)targetSpeed,outMin,outMax);
	  
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

void RegulatedMotor::setState(int _state){
	//lastState = state;
	state = _state;	
}

long RegulatedMotor::getEncoder(){
  return *encoder;
}

int RegulatedMotor::getError(){
  return error;
}