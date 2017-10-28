// TODO: finish unit conversion
#include <common/math/math.h>

#include "ArcTurn.h"


ArcTurn::ArcTurn(Direction dir) : Command("RealArcTurn"), mouse(RealMouse::inst()), dir(dir) {}

void ArcTurn::initialize() {
  curPose = mouse->getGlobalPose();
  curCol = mouse->getCol();
  curRow = mouse->getRow();
  curDir = mouse->getDir();

  startPose = curPose;
  startCol = curCol;
  startRow = curRow;

  goalYaw = dir_to_yaw(dir);

  // determine vtc of arc
  vtc_x = curCol * smartmouse::maze::UNIT_DIST_M + smartmouse::maze::HALF_UNIT_DIST;
  vtc_y = curRow * smartmouse::maze::UNIT_DIST_M + smartmouse::maze::HALF_UNIT_DIST;
  switch (curDir) {
    case Direction::N: {
      vtc_y += smartmouse::maze::HALF_UNIT_DIST;
      if (dir == Direction::E) {
        vtc_x += smartmouse::maze::HALF_UNIT_DIST;
      } else {
        vtc_x -= smartmouse::maze::HALF_UNIT_DIST;
      }
      break;
    }
    case Direction::E: {
      vtc_x -= smartmouse::maze::HALF_UNIT_DIST;
      if (dir == Direction::S) {
        vtc_y += smartmouse::maze::HALF_UNIT_DIST;
      } else {
        vtc_y -= smartmouse::maze::HALF_UNIT_DIST;
      }
      break;
    }
    case Direction::S: {
      vtc_y -= smartmouse::maze::HALF_UNIT_DIST;
      if (dir == Direction::W) {
        vtc_x -= smartmouse::maze::HALF_UNIT_DIST;
      } else {
        vtc_x += smartmouse::maze::HALF_UNIT_DIST;
      }
      break;
    }
    case Direction::W: {
      vtc_x += smartmouse::maze::HALF_UNIT_DIST;
      if (dir == Direction::N) {
        vtc_y -= smartmouse::maze::HALF_UNIT_DIST;
      } else {
        vtc_y += smartmouse::maze::HALF_UNIT_DIST;
      }
      break;
    }
    default:
      exit(0);
  }
}

void ArcTurn::execute() {

  double cur_x = fabs(curPose.row - vtc_x);
  double cur_y = fabs(curPose.col - vtc_y);

  double dAngle;
  if ((dir == Direction::N) || (dir == Direction::S)) {
    dAngle = atanf(fabs(cur_x / cur_y));
  } else {
    dAngle = atanf(fabs(cur_y / cur_x));
  }

  double ang_error = smartmouse::math::yawDiff(dAngle, fabs(smartmouse::math::yawDiff(curPose.yaw, dir_to_yaw(curDir))));
  double arc_error = (smartmouse::maze::HALF_UNIT_DIST / pose_dist(curPose, vtc_x, vtc_y)) - 1;
  double corr = (ang_error * ang_weight) + (arc_error * arc_weight);

  double fast_speed = FAST_ARC_SPEED * ((corr * -kp_turn) + 1);
  double slow_speed = SLOW_ARC_SPEED * ((corr * kp_turn) + 1);

  if (right_of_dir(curDir) == dir) {
    mouse->setSpeedCps(fast_speed, slow_speed);
  } else {
    mouse->setSpeedCps(slow_speed, fast_speed);
  }
}

bool ArcTurn::isFinished() {
  curPose = mouse->getGlobalPose();
  curCol = mouse->getCol();
  curRow = mouse->getRow();
  curDir = mouse->getDir();

  dYaw = smartmouse::math::yawDiff(curPose.yaw, goalYaw);
  return (curCol != startCol) || (curRow != startRow);
}

void ArcTurn::end() {
  mouse->internalTurnToFace(dir);
}

double ArcTurn::pose_dist(GlobalPose pose, double x, double y) {
  return sqrtf(powf(pose.col - x, 2) + powf(pose.row - y, 2));
}
