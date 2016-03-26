#include "KinematicController.h"
#include <math.h>

KinematicController::KinematicController(RegulatedMotor* leftMotor,
    RegulatedMotor* rightMotor,
    int leftMotorDirection, int rightMotorDirection,
    float trackWidth, float wheelDiameter, float encoderCPR)
  : leftMotor(leftMotor),
  rightMotor(rightMotor),
  leftMotorDirection(leftMotorDirection),
  rightMotorDirection(rightMotorDirection),
  wheelDiameter(wheelDiameter),
  trackWidth(trackWidth),
  encoderCPR(encoderCPR),
  sampleTime(5) {
    setSampleTime(sampleTime);
  }

void KinematicController::setup(){
  leftMotor->setup();
  rightMotor->setup();
}

void KinematicController::setDriveBaseProperties(uint16_t trackWidth, uint16_t wheelDiameter){
  this->trackWidth = trackWidth;
  this->wheelDiameter = wheelDiameter;
}

void KinematicController::setSampleTime(unsigned long sampleTime){
  leftMotor->setSampleTime(sampleTime);
  rightMotor->setSampleTime(sampleTime);
}

void KinematicController::setAcceleration(unsigned int forwardAcceleration,
    float ccwAcceleration, unsigned int forwardDeceleration,
    float ccwDeceleration){

  this->forwardAcceleration = mmToTick(forwardAcceleration);
  this->ccwAcceleration = radToTick(ccwAcceleration);
  this->forwardDeceleration = mmToTick(forwardDeceleration);
  this->ccwDeceleration = radToTick(ccwDeceleration);

  forwardAccelerationStep = forwardAcceleration * sampleTime / 1000;
  forwardDecelerationStep = forwardDeceleration * sampleTime / 1000;
  ccwAccelerationStep = ccwAcceleration * sampleTime;
  ccwDecelerationStep = ccwDeceleration * sampleTime;

  Serial.println(forwardAcceleration);
  Serial.println(forwardAccelerationStep);
}


boolean KinematicController::run(){
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - lastRunTime;

  if (deltaTime >= sampleTime){

    standby = true;

    if (currentTime - lastRamp < STANDBY_DELAY) standby = false;

    long forwardOutput = 0;
    long ccwOutput = 0;

    if (state == ControllerState::VELOCITY){
      if (lastForwardVelocity == targetForwardVelocity) {
        forwardOutput = targetForwardVelocity;
      }
      else {
        forwardOutput = speedRamp(lastForwardVelocity, targetForwardVelocity, forwardAccelerationStep, forwardDecelerationStep);
        standby = false;
        lastRamp = currentTime;
      }

      if (lastCCWVelocity == targetCCWVelocity) {
        ccwOutput = targetCCWVelocity;
      }
      else {
        ccwOutput = speedRamp(lastCCWVelocity, targetCCWVelocity, ccwAccelerationStep, ccwDecelerationStep);
        standby = false;
        lastRamp = currentTime;
      }

      leftMotor->setSpeed(calculateLeftWheelSpeed(forwardOutput, ccwOutput));
      rightMotor->setSpeed(calculateRightWheelSpeed(forwardOutput, ccwOutput));

      leftMotor->runNow(deltaTime);
      rightMotor->runNow(deltaTime);

      lastForwardVelocity = forwardOutput;
      lastCCWVelocity = ccwOutput;
    } else if (state == ControllerState::OFF) {
      lastForwardVelocity = 0;
      lastCCWVelocity = 0;
    }

    long localX = this->getOdometryForward();
    long localTheta = this->getOdometryCCW();
    globalX += (float)(localX-lastLocalX)*cos(localTheta*M_PI/180);
    globalY += (float)(localX-lastLocalX)*sin(localTheta*M_PI/180);
    lastRunTime = currentTime;
    lastLocalX = localX;
    lastLocalTheta = localTheta;

    return true;
  }

  return false;
}

void KinematicController::getGlobalPosition(long *x, long *y){
  *x = round(globalX);
  *y = round(globalY);
}

void KinematicController::setVelocity(int forwardVelocity, float ccwVelocity){
  state = ControllerState::VELOCITY;
  standby = false;
  targetForwardVelocity = mmToTick(forwardVelocity);
  targetCCWVelocity = radToTick(ccwVelocity);
}

void KinematicController::travel(int forwardDistance, float ccwAngle, unsigned int forwardSpeed, float ccwSpeed){
  state = ControllerState::POSITION;
  _travel(forwardDistance, ccwAngle, forwardSpeed, ccwSpeed);
}

void KinematicController::brake(){
  state = ControllerState::OFF;
  leftMotor->setState(RegulatedMotor::MotorState::BRAKE);
  rightMotor->setState(RegulatedMotor::MotorState::BRAKE);
}

void KinematicController::coast(){
  state = ControllerState::OFF;
  leftMotor->setState(RegulatedMotor::MotorState::COAST);
  rightMotor->setState(RegulatedMotor::MotorState::COAST);
}

void KinematicController::_travel(int forwardDistance, float ccwAngle, unsigned int forwardSpeed, float ccwSpeed){
  originTime = millis();
  originForwardTick = calculateForwardTick();
  originCCWTick = calculateCCWTick();
  targetForwardTick = mmToTick(forwardDistance);
  targetCCWTick = radToTick(ccwAngle);
  positionForwardVelocity = forwardSpeed;
  positionCCWVelocity = ccwSpeed;
}


long KinematicController::mmToTick(long mm){
  return (mm*encoderCPR)/(M_PI*wheelDiameter);
}

long KinematicController::radToTick(float rad){
  return mmToTick(rad*trackWidth);
}

long KinematicController::calculateLeftWheelSpeed(long forwardVelocity, long ccwVelocity){
  long s = (forwardVelocity - ccwVelocity)*leftMotorDirection;
  return s;
}

long KinematicController::calculateRightWheelSpeed(long forwardVelocity, long ccwVelocity){
  long s = (forwardVelocity + ccwVelocity)*rightMotorDirection;
  return s;
}

long KinematicController::calculateForwardTick(){
  return (leftMotor->getEncoder()*leftMotorDirection + rightMotor->getEncoder()*rightMotorDirection)/2;
}

long KinematicController::calculateCCWTick(){
  return (-leftMotor->getEncoder()*leftMotorDirection + rightMotor->getEncoder()*rightMotorDirection);
}

long KinematicController::speedRamp(long last, long target,long up, long down){
  long ret;
  if (last >= 0){
    if (target > last){
      ret = last + up;
      if (ret > target) ret = target;
    } else {
      ret = last - down;
      if (ret < target) ret = target;
    }
  } else { // last < 0
    if (target > last){
      ret = last + down;
      if (ret > target) ret = target;
    } else {
      ret = last - up;
      if (ret < target) ret = target;
    }
  }
  return ret;
}

boolean KinematicController::isStandby(){
  return standby;
}

long KinematicController::getOdometryForward(){
  return calculateForwardTick()* (M_PI*wheelDiameter) / encoderCPR;
}

long KinematicController::getOdometryCCW(){
  return (calculateCCWTick() * (M_PI*wheelDiameter) / encoderCPR)/(trackWidth*M_PI/180);
}
