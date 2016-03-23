#ifndef RegulatedMotor_h
#define RegulatedMotor_h
#define LIBRARY_VERSION	2.0.0
#include "../Encoder/Encoder.h"

#define MOTORSTATE_PWM	0
#define MOTORSTATE_SPEED	1
#define MOTORSTATE_BRAKE	3
#define MOTORSTATE_COAST	4

class RegulatedMotor
{
public:
	RegulatedMotor(long* encoder, int fwdPin, int revPin);
	bool run();
	void setSpeed(int speed);
	void setState(int state);
	void setPID(float kp, float ki, float kd, float kvff);
	void setSampleTime(unsigned long sampleTime);
	void goPWM(int pwm);
	long getEncoder();
	int getError();
private:
	long* encoder;

	long thisPosition;
	long lastPosition;

	int calculatedSpeed;
	int lastCalculatedSpeed;
	int targetSpeed;

	int outputValue;

	int speedScale;

	float kp;
	float ki;
	float kd;
	float kvff;

	int iTerm;

	unsigned long lastTime;

	unsigned long sampleTime;

	int state;
	int lastState;

	int fwdPin;
	int revPin;
	int pwmPin;

	int error;
	//int getPWM(int speed);

};
#endif
