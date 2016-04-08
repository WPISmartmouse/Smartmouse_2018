#include "Forward.h"
#include "LEDBlink.h"
#include <math.h>
#include <cfloat>

Forward::Forward() : mouse(RealMouse::inst()), checkedWalls(false),
  wallOnLeft(true), wallOnRight(true), state(FwdState::GO_UNTIL_CHECK),
  dispError(FLT_MAX) {}

void Forward::initialize(){
  start = mouse->getPose();
  disp = 0.0f;
  goalYaw = toYaw(mouse->getDir());
}

float Forward::forwardDisplacement(Pose p0, Pose p1){
  switch(mouse->getDir()){
    case Direction::N:
      return p1.y - p0.y;
      break;
    case Direction::E:
      return p1.x - p0.x;
      break;
    case Direction::S:
      return p0.y - p1.y;
      break;
    case Direction::W:
      return p0.x - p1.x;
      break;
  }
}

float Forward::yawDiff(float y1, float y2){
  float diff = y2 - y1;
  if (diff > M_PI) return diff - M_PI*2;
  if (diff < -M_PI) return diff + M_PI*2;
  return diff;
}

bool Forward::outOfRange(float range){
  return (range >= 0.12); //dist at which it will consider a wall
}

void Forward::execute(){
  float currentYaw = -999;
  float dYaw = -999;
  int imu_in_calib = (mouse->getIMUCalibration() > 0);
  if (imu_in_calib) {
    currentYaw = mouse->getIMUYaw();
    mouse->updateGlobalYaw();
    // TODO: not query imu calib inside updateGlobalYaw
  } else {
    currentYaw = mouse->getPose().yaw;
  }

  dYaw = yawDiff(currentYaw, goalYaw);
  float angleError = -dYaw;

  distances = mouse->getRawDistances();
  //TODO FIX UNITS
  disp = forwardDisplacement(start, mouse->getPose()) / 1000.0;

  float dToWallRight = distances[0] * cos(M_PI/6 + angleError);
  float dToWallLeft = distances[2] * cos(M_PI/6 - angleError);


  float rightWallError = RealMouse::WALL_DIST_SETPOINT - dToWallRight;
  float leftWallError = RealMouse::WALL_DIST_SETPOINT - dToWallLeft;

  dispError = 0;
  switch(state) {
    case FwdState::GO_UNTIL_CHECK:
      if (disp >= AbstractMaze::UNIT_DIST/2) {
        state = FwdState::CHECK;
      }
      dispError = AbstractMaze::UNIT_DIST - disp;
      break;

    case FwdState::CHECK: {

      Direction right = right_of_dir(mouse->getDir());
      Direction left = left_of_dir(mouse->getDir());
      walls[static_cast<int>(right)] = dToWallRight < RealMouse::WALL_DIST;
      walls[static_cast<int>(left)] = dToWallLeft < RealMouse::WALL_DIST;

      if (distances[1] > minFrontDist && distances[1] < maxFrontDist) {
        addParallel(new LEDBlink(LEDBlink::B, 300));
        state = FwdState::STOP_AT_WALL;
      }
      else {
        state = FwdState::STOP_AT_DIST;
      }
      dispError = AbstractMaze::UNIT_DIST - disp;
      break;
    }

    case FwdState::STOP_AT_WALL:
      dispError = distances[1] - distFromSensorToWallFromCenter;
      break;

    case FwdState::STOP_AT_DIST:
      dispError = AbstractMaze::UNIT_DIST - disp;
      break;
  }

  float correction = 0.0;
  if (!outOfRange(distances[2])) {
    digitalWrite(RealMouse::LEDG, 1);
    digitalWrite(RealMouse::LEDR, 0);
    correction = -leftWallError * kPWall * dispError;
  }
  else if (!outOfRange(distances[0])) {
    digitalWrite(RealMouse::LEDG, 0);
    digitalWrite(RealMouse::LEDR, 1);
    correction = rightWallError * kPWall * dispError;
  }
  else {
    digitalWrite(RealMouse::LEDR, 0);
    digitalWrite(RealMouse::LEDG, 0);
  }

  float sumCorrection = correction + dYaw * kYaw;
  float speed = dispError * kPDisp;

  if (sumCorrection > RealMouse::MAX_ROT_SPEED) {
    sumCorrection = RealMouse::MAX_ROT_SPEED;
  }
  else if (sumCorrection < -RealMouse::MAX_ROT_SPEED) {
    sumCorrection = -RealMouse::MAX_ROT_SPEED;
  }

  speed = speed > RealMouse::MAX_SPEED ? RealMouse::MAX_SPEED : speed;
  speed = speed < RealMouse::MIN_SPEED ? RealMouse::MIN_SPEED : speed;

  mouse->setSpeed(speed, sumCorrection);
}

bool Forward::isFinished(){
  if (dispError <= 0) {
    return true;
  }
  return false;

}

void Forward::end(){
  mouse->internalForward();
  mouse->setSpeed(0,0);

  walls[static_cast<int>(mouse->getDir())] = distances[1] < RealMouse::WALL_DIST;
  walls[static_cast<int>(opposite_direction(mouse->getDir()))] = false;

  mouse->suggestWalls(walls);
}
