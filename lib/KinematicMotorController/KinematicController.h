#pragma once

#include "RegulatedMotor.h"
#include <Arduino.h>

#define round(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))

class KinematicController{
  public:

    // \brief units for everything is in millimeters and radians
    KinematicController(RegulatedMotor* leftMotor, RegulatedMotor* rightMotor,
        int leftMotorDirection, int rightMotorDirection,
        float trackWidth, float wheelDiameter, float encoderCPR);

    // \brief you have to call this in setup
    void setup();

    // \brief units in mm/s^2 rad/s^2
    void setAcceleration(unsigned int forwardAcceleration, float ccwAcceleration,
        unsigned int forwardDeceleration, float ccwDeceleration);

    // \brief units in mm
    void setDriveBaseProperties(uint16_t trackWidth, uint16_t wheelDiameter);

    // \brief units in mm/s an radians/s
    void setVelocity(int forwardVelocity, float ccwVelocity);

    // \brief units in mm, radians, mm/s an radians/s
    void travel(int forwardDistance, float ccwAngle, unsigned int forwardSpeed,
        float ccwSpeed);

    void brake();
    void coast();

    boolean run();

    long calculateForwardTick();
    long calculateCCWTick();

    long getOdometryForward();
    long getOdometryCCW();

    void getGlobalPosition(long *x, long *y);
    boolean isStandby();

    void setSampleTime(unsigned long sampleTime);

  private:

    enum class ControllerState {COAST, OFF, POSITION, VELOCITY};

    boolean standby = true;
    unsigned long lastRamp;

    RegulatedMotor* leftMotor;
    RegulatedMotor* rightMotor;

    const int kp = 10;
    unsigned long sampleTime; //milliseconds

    int leftMotorDirection;
    int rightMotorDirection;

    float trackWidth;
    float wheelDiameter;
    float encoderCPR;

    unsigned long forwardAcceleration;	//tick*s^-2
    unsigned long ccwAcceleration;	//rad*s^-2
    unsigned long forwardDeceleration;	//tick*s^-2
    unsigned long ccwDeceleration;	//rad*s^-2

    unsigned long forwardAccelerationStep;
    float ccwAccelerationStep;
    unsigned long forwardDecelerationStep;
    float ccwDecelerationStep;

    ControllerState state;

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

    void _travel(int forwardDistance, float ccwAngle, unsigned int forwardSpeed, float ccwSpeed);

    long calculateLeftWheelSpeed(long forwardVelocity, long ccwVelocity);
    long calculateRightWheelSpeed(long forwardVelocity, long ccwVelocity);

    long mmToTick(long mm);
    long radToTick(float rad);

    long speedRamp(long last, long target,long up, long down);

    static const int STANDBY_DELAY = 500;
    static const int STANDBY_TOLERANCE = 4;
};
