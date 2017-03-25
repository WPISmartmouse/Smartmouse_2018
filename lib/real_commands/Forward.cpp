#include "Forward.h"
#include "LEDBlink.h"
#include <math.h>
#include <cfloat>

Forward::Forward() :
  mouse(RealMouse::inst()),
  checkedWalls(false),
  state(FwdState::GO_UNTIL_CHECK),
  remainingDistance(FLT_MAX),
  walls{true, true, true, true},
  goalYaw(0) {}

void Forward::initialize(){
  start = mouse->getPose();
  distanceSoFar = 0.0f;
  goalYaw = toYaw(mouse->getDir());
}

float Forward::forwardDisplacement(Pose p0, Pose p1){
  switch(mouse->getDir()){
    case Direction::N:
      return p1.y - p0.y;
    case Direction::E:
      return p1.x - p0.x;
    case Direction::S:
      return p0.y - p1.y;
    case Direction::W:
      return p0.x - p1.x;
  }
}

float Forward::yawDiff(){
  float currentYaw;

  int imu_in_calib = (mouse->getIMUCalibration() > 0);
  if (imu_in_calib) {
    currentYaw = mouse->getIMUYaw();
    mouse->updateGlobalYaw();
  } else {
    currentYaw = mouse->getPose().yaw;
  }

  float diff = currentYaw - goalYaw;
  if (diff > M_PI) return diff - M_PI*2;
  if (diff < -M_PI) return diff + M_PI*2;
  return diff;
}

float Forward::calculateRemainingDistance(float dToWallLeft, float dToWallRight, float rawFrontWallDist){
  switch(this->state) {
    case FwdState::GO_UNTIL_CHECK:
      if (distanceSoFar >= CHECK_DIST) {
        this->state = FwdState::CHECK;
      }
      return AbstractMaze::UNIT_DIST - this->distanceSoFar;

    case FwdState::CHECK: {

      Direction right = right_of_dir(mouse->getDir());
      Direction left = left_of_dir(mouse->getDir());
      this->walls[static_cast<int>(right)] = dToWallRight < RealMouse::WALL_DIST;
      this->walls[static_cast<int>(left)] = dToWallLeft < RealMouse::WALL_DIST;

      if (rawFrontWallDist < maxFrontDist) {
        digitalWrite(RealMouse::LEDGO, 1);
        this->state = FwdState::STOP_AT_WALL;
      }
      else {
        this->state = FwdState::STOP_AT_DIST;
      }
      return AbstractMaze::UNIT_DIST - distanceSoFar;
    }

    case FwdState::STOP_AT_WALL:
      return rawFrontWallDist - distFromSensorToWallFromCenter;

    case FwdState::STOP_AT_DIST:
      return AbstractMaze::UNIT_DIST - distanceSoFar;
  }
}

bool Forward::outOfRange(float range){
  return (range >= WALL_OUT_OF_RANGE_DIST);
}

void Forward::execute(){
  float dYaw = yawDiff();
  float angleError = -dYaw;

  this->distances = mouse->getRawDistances();
  this->distanceSoFar = forwardDisplacement(start, mouse->getPose());

  float rawRightWallDist = distances[0];
  float rawFrontWallDist = distances[1];
  float rawLeftWallDist = distances[2];

  float dToWallRight = rawRightWallDist * cos(RealMouse::SENSOR_ANGLE + angleError);
  float dToWallLeft = rawLeftWallDist * cos(RealMouse::SENSOR_ANGLE - angleError);

  this->remainingDistance = calculateRemainingDistance(dToWallLeft, dToWallRight, rawFrontWallDist);

  float rightWallError = RealMouse::WALL_DIST_SETPOINT - dToWallRight;
  float leftWallError = RealMouse::WALL_DIST_SETPOINT - dToWallLeft;

  float correction = 0.0;
  if (!outOfRange(rawLeftWallDist)) {
    digitalWrite(RealMouse::LEDG, 1);
    digitalWrite(RealMouse::LEDR, 0);
    correction = -leftWallError * kPWall * this->remainingDistance;
  }
  else if (!outOfRange(rawRightWallDist)) {
    digitalWrite(RealMouse::LEDG, 0);
    digitalWrite(RealMouse::LEDR, 1);
    correction = rightWallError * kPWall * this->remainingDistance;
  }
  else {
    digitalWrite(RealMouse::LEDR, 0);
    digitalWrite(RealMouse::LEDG, 0);
  }

  float sumCorrection = correction;
  float speed = this->remainingDistance * kPDisp;

  //don't allow the fwd command to set forward speed to zero
  if (speed < MIN_SPEED) speed = MIN_SPEED;

  mouse->setSpeed(speed, sumCorrection);
}

bool Forward::isFinished(){
  if (remainingDistance <= 0) {
    return true;
  }
  return false;

}

void Forward::end(){
  digitalWrite(RealMouse::LEDGO, 0);
  mouse->internalForward();
  mouse->setSpeed(0,0);

  this->walls[static_cast<int>(mouse->getDir())] = this->distances[1] < RealMouse::WALL_DIST;
  this->walls[static_cast<int>(opposite_direction(mouse->getDir()))] = false;

  mouse->suggestWalls(this->walls);
}
