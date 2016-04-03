#include "Forward.h"
#include <math.h>

Forward::Forward() : mouse(RealMouse::inst()), checkedWalls(false), wallOnLeft(true), wallOnRight(true){}

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
  return (range >= 0.254);
}

void Forward::execute(){
  float currentYaw = -999;
  float dYaw = -999;
  int imu_in_calib = (mouse->getIMUCalibration() == 3);
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

  float dispError = AbstractMaze::UNIT_DIST - disp;

  float dToWallRight = distances[0] * cos(M_PI/6 + angleError);
  float dToWallLeft = distances[2] * cos(M_PI/6 - angleError);

  float speed = dispError * kPDisp + copysignf(1.0, dispError) * minimalSpeed;

  float rightWallError = RealMouse::WALL_DIST_SETPOINT - dToWallRight;
  float leftWallError = RealMouse::WALL_DIST_SETPOINT - dToWallLeft;

  speed = speed > RealMouse::MAX_SPEED ? RealMouse::MAX_SPEED : speed;
  speed = speed < RealMouse::MIN_SPEED ? RealMouse::MIN_SPEED : speed;

  if (outOfRange(distances[0]) && wallOnRight){
    wallOnRight = false;
  }
  if (outOfRange(distances[2]) && wallOnLeft){
    wallOnLeft = false;
  }

  float correction = 0.0;
  if (wallOnLeft) {
    correction = -leftWallError * kPWall * dispError;
  }
  else if (wallOnRight) {
    correction = rightWallError * kPWall * dispError;
  }

  else if (correction > RealMouse::MAX_ROT_SPEED) {
    correction = RealMouse::MAX_ROT_SPEED;
  }
  else if (correction < -RealMouse::MAX_ROT_SPEED) {
    correction = -RealMouse::MAX_ROT_SPEED;
  }

  if (!checkedWalls && dispError < AbstractMaze::UNIT_DIST/2) {
    checkedWalls = true;
    walls[static_cast<int>(right_of_dir(mouse->getDir()))] =
      dToWallRight < RealMouse::WALL_DIST;
    walls[static_cast<int>(left_of_dir(mouse->getDir()))] =
      dToWallLeft < RealMouse::WALL_DIST;
  }

  if (disp > ignore_wall_region_L &&
    disp < ignore_wall_region_H) {
      correction = 0;
  }
  correction = 0;
  float sumCorrection = correction + dYaw * kYaw;

  Serial.print(" leftWallError:");
  Serial.print(leftWallError);
  Serial.print(" dispError:");
  Serial.print(dispError);
  Serial.print(" correction");
  Serial.println(correction);
  Serial1.print(" currentYaw");
  Serial1.print(currentYaw);
  Serial1.print(" goalYaw");
  Serial1.print(goalYaw);
  Serial1.print(" dYaw");
  Serial1.println(dYaw);

  mouse->setSpeed(speed, sumCorrection);

  Pose currentPose = mouse->getPose();
  mouse->clearDisplay();
  mouse->display.println(currentPose.x);
  mouse->display.println(currentPose.y);
  mouse->display.println(currentPose.yaw);
  mouse->updateDisplay();
}

bool Forward::isFinished(){
  return disp >= AbstractMaze::UNIT_DIST;
}

void Forward::end(){
  mouse->internalForward();
  mouse->setSpeed(0,0);

  walls[static_cast<int>(mouse->getDir())] = distances[1] < RealMouse::WALL_DIST;
  walls[static_cast<int>(opposite_direction(mouse->getDir()))] = false;

  mouse->suggestWalls(walls);
}
