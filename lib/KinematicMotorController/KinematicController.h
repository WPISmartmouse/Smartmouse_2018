#ifndef KinematicController_h
#define KinematicController_h
#include "RegulatedMotor.h"
#include "Arduino.h"
#define KINEMATIC_OFF	0
#define KINEMATIC_VELOCITY	1
#define KINEMATIC_POSITION 2
#define STANDBY_TOLERANCE 4
#define STANDBY_DELAY 500
#define round(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))

#define MAXIMUM_DISTANCE 10000

class KinematicController{
public:
	KinematicController(RegulatedMotor* leftMotor, RegulatedMotor* rightMotor,
		int leftMotorDirection, int rightMotorDirection,
		float wheelDistance, float wheelDiameter, unsigned int encoderCPR);

	void setAcceleration(unsigned int forwardAcceleration, unsigned int ccwAcceleration, unsigned int forwardDeceleration, unsigned int ccwDeceleration);
	void calibrate(uint16_t wheelDistance, uint16_t wheelDiameter);
	void goVelocity(int forwardVelocity, int ccwVelocity);
	void goPosition(int forwardDistance, int ccwAngle, unsigned int forwardSpeed, unsigned int ccwSpeed);
	
	void brake();
	void coast();

	boolean run();

	long calculateForwardTick();
	long calculateCCWTick();

	long getOdometryForward();
	long getOdometryCCW();	

	void getGlobalPosition(long *x, long *y);
	boolean isStandby();


private:
	boolean standby = true;
	unsigned long lastRamp;

	RegulatedMotor* leftMotor;
	RegulatedMotor* rightMotor;

	const int kp = 10;
	int sampleTime = 50;

	int leftMotorDirection; 
	int rightMotorDirection;

	float wheelDistance;
	float wheelDiameter;
	unsigned int encoderCPR;

	unsigned long forwardAcceleration;	//tick*s^-2
	unsigned long ccwAcceleration;	//tick*s^-2
	unsigned long forwardDeceleration;	//tick*s^-2
	unsigned long ccwDeceleration;	//tick*s^-2

	unsigned long atomicForwardAcceleration;
	unsigned long atomicCCWAcceleration;
	unsigned long atomicForwardDeceleration;
	unsigned long atomicCCWDeceleration;

	int state;


	long targetForwardVelocity;	//tick/s
	long targetCCWVelocity;	//tick/s
	long lastForwardVelocity = 0;	//tick/s
	long lastCCWVelocity = 0;	//tick/s

	long targetForwardTick;
	long targetCCWTick;
	long positionForwardVelocity;
	long positionCCWVelocity;

	unsigned long originTime;
	long originForwardTick;
	long originCCWTick;

	unsigned long lastRunTime;

	long lastLocalX = 0;
	long lastLocalTheta = 0;

	float globalX = 0;
	float globalY = 0;

	void _goPosition(int forwardDistance, int ccwAngle, unsigned int forwardSpeed, unsigned int ccwSpeed);

	long calculateLeftWheelSpeed(long forwardVelocity, long ccwVelocity);
	long calculateRightWheelSpeed(long forwardVelocity, long ccwVelocity);

	long mmToTick(long mm);
	long degToTick(long deg);

	long speedRamp(long last, long target,long up, long down);
};

#endif