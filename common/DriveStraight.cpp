#include <common/KinematicController/KinematicController.h>
#include <limits>
#include "DriveStraight.h"

#ifdef ARDUINO
#include <Arduino.h>
#endif

const double DriveStraight::kPWall = 0.80; //TODO: Should be 0.8
const double DriveStraight::kDWall = 50;
const double DriveStraight::kPYaw = 7.0;

DriveStraight::DriveStraight() : disp(0.0), goalDisp(AbstractMaze::UNIT_DIST), dispError(goalDisp) {}

void DriveStraight::start(GlobalPose start_pose, double goalDisp) {
  this->goalDisp = goalDisp;
  this->start_pose = start_pose;
}

std::pair<double, double> DriveStraight::compute_wheel_velocities(Mouse *mouse) {
  GlobalPose current_pose = mouse->getGlobalPose();
  disp = fwdDisp(mouse->getDir(), current_pose, start_pose);
  dispError = goalDisp - disp;

  double errorToCenter = sidewaysDispToCenter(mouse);
  double goalYaw = dir_to_yaw(mouse->getDir()) + errorToCenter * kPYaw;

  // The goal is to be facing straight when you wall distance is correct.
  // To achieve this, we control our yaw as a function of our error in wall distance
  double yawError = KinematicController::yawDiff(goalYaw, current_pose.yaw);

  // given starting velocity, fixed acceleration, and final velocity
  // generate the velocity profile for achieving this as fast as possible
  if (current_speed < config.MAX_SPEED) {
    l = current_speed + acceleration;
  }
  else if (dispError < dispToSlowDown) {
    l = current_speed - acceleration;
  }
  double l = ; // config.MAX_SPEED;
  double r = velocity_profile(); // config.MAX_SPEED;
  double correction = kPWall * yawError;

  if (yawError < 0) { // need to turn left
    l += correction; // correction will be negative here
  } else {
    r -= correction;
  }

  return std::pair<double, double>(l, r);
}

double DriveStraight::fwdDisp(Direction dir, GlobalPose current_pose, GlobalPose start_pose) {
  switch (dir) {
    case Direction::N:return start_pose.y - current_pose.y;
    case Direction::E:return current_pose.x - start_pose.x;
    case Direction::S:return current_pose.y - start_pose.y;
    case Direction::W:return start_pose.x - current_pose.x;
    default:return std::numeric_limits<double>::quiet_NaN();
  }
}

double DriveStraight::dispToNextEdge(Mouse *mouse) {
  GlobalPose current_pose = mouse->getGlobalPose();
  Direction dir = mouse->getDir();

  switch (dir) {
    case Direction::N: {
      double next_row_y = mouse->getRow() * AbstractMaze::UNIT_DIST;
      return current_pose.y - next_row_y;
    }
    case Direction::S: {
      double next_row_y = (mouse->getRow() + 1) * AbstractMaze::UNIT_DIST;
      return next_row_y - current_pose.y;
    }
    case Direction::E: {
      double next_col_x = (mouse->getCol() + 1) * AbstractMaze::UNIT_DIST;
      return next_col_x - current_pose.x;
    }
    case Direction::W: {
      double next_col_x = mouse->getCol() * AbstractMaze::UNIT_DIST;
      return current_pose.x - next_col_x;
    }
    default:return std::numeric_limits<double>::quiet_NaN();
  }
}

double DriveStraight::dispToNthEdge(Mouse *mouse, unsigned int n) {
  // give the displacement to the nth edge like above...
  return 0;
}

double DriveStraight::sidewaysDispToCenter(Mouse *mouse) {
  // local y is sideways, increasing from left to right
  return mouse->getLocalPose().to_left - AbstractMaze::HALF_UNIT_DIST;
}

double DriveStraight::fwdDispToCenter(Mouse *mouse) {
  return AbstractMaze::HALF_UNIT_DIST - mouse->getLocalPose().to_back;
}

double DriveStraight::fwdDispToDiag(Mouse *mouse) {
  return (AbstractMaze::HALF_UNIT_DIST - (config.TRACK_WIDTH / 2.0)) - mouse->getLocalPose().to_back;
}
