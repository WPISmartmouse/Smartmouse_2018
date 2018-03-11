//TODO: finish unit conversion

#include <common/math/math.h>
#include "ArcTurn.h"

ArcTurn::ArcTurn(Direction dir) : Command("SimArcTurn"), mouse(SimMouse::inst()), dir(dir) {}

void ArcTurn::initialize() {
  mouse->kinematic_controller.enable_sensor_pose_estimate = false;
  setTimeout(2000);
  goalYaw = dir_to_yaw(dir);
  startPose = mouse->getGlobalPose();

  // determine vtc point of arc
  // inital center of sqaure

  startCol = mouse->getCol();
  startRow = mouse->getRow();
  vtc_x = mouse->getCol()*smartmouse::maze::UNIT_DIST_M + smartmouse::maze::HALF_UNIT_DIST;
  vtc_y = mouse->getRow()*smartmouse::maze::UNIT_DIST_M + smartmouse::maze::HALF_UNIT_DIST;

  end_x = vtc_x;
  end_y = vtc_y;

  // move to corner
  switch (mouse->getDir()) {
    case Direction::N: {
      vtc_y += smartmouse::maze::HALF_UNIT_DIST;
      if (dir == Direction::E){
        vtc_x += smartmouse::maze::HALF_UNIT_DIST;
      } else {
        vtc_x -= smartmouse::maze::HALF_UNIT_DIST;
      }
      end_x = vtc_x;
      break;
    }
    case Direction::E: {
      vtc_x -= smartmouse::maze::HALF_UNIT_DIST;
      if (dir == Direction::S) {
        vtc_y += smartmouse::maze::HALF_UNIT_DIST;
      } else {
        vtc_y -= smartmouse::maze::HALF_UNIT_DIST;
      }
      end_y = vtc_y;
      break;
    }
    case Direction::S: {
      vtc_y -= smartmouse::maze::HALF_UNIT_DIST;
      if (dir == Direction::W){
        vtc_x -= smartmouse::maze::HALF_UNIT_DIST;
      } else {
        vtc_x += smartmouse::maze::HALF_UNIT_DIST;
      }
      end_x = vtc_x;
      break;
    }
    case Direction::W: {
      vtc_x += smartmouse::maze::HALF_UNIT_DIST;
      if (dir == Direction::N){
        vtc_y -= smartmouse::maze::HALF_UNIT_DIST;
      } else {
        vtc_y += smartmouse::maze::HALF_UNIT_DIST;
      }
      end_y = vtc_y;
      break;
    }
    default:
      exit(0);
  }

  //print("%f,%f\n\r", vtc_x, vtc_y);
}

void ArcTurn::execute() {

  GlobalPose curPose = mouse->getGlobalPose();
  double cur_x = fabs(curPose.col - vtc_x);
  double cur_y = fabs(curPose.row - vtc_y);

  double dAngle;
  if ((dir == Direction::N)||(dir == Direction::S)) {
    dAngle = atanf(fabs(cur_x/cur_y));
  } else {
    dAngle = atanf(fabs(cur_y/cur_x));
  }

  double ang_error = fabs(smartmouse::math::yaw_diff(curPose.yaw, dir_to_yaw(mouse->getDir())))-dAngle;
  double arc_error = (smartmouse::maze::HALF_UNIT_DIST/pose_dist(mouse->getGlobalPose(), vtc_x, vtc_y))-1;

  double corr = (ang_error*1.0)+(arc_error*0.5);
//  print("%f\t%f\n\r", dAngle, smartmouse::math::yaw_diff(curPose.yaw, dir_to_yaw(mouse->getDir())));

  double fast_speed = FAST_ARC_SPEED*((corr*-5)+1);
  double slow_speed = SLOW_ARC_SPEED*((corr*5)+1);

  if (right_of_dir(mouse->getDir()) == dir ){
    mouse->setSpeedCps(fast_speed, slow_speed);
  } else {
    mouse->setSpeedCps(slow_speed, fast_speed);
  }
  // this allows us to use that
  if (fabs(dYaw) < 0.1 && mouse->kinematic_controller.enable_sensor_pose_estimate) {
    //digitalWrite(RealMouse::LED_2, 1);
    // FIXME: this is kind of a hack. It's needed because WallFollower checks dir in order to compute
    // FIXME: the correct yaw. it adds dir_to_yaw(getDir()), so we must assume we're close enough
    //mouse->kinematic_controller.enable_sensor_pose_estimate = false;
    //mouse->internalTurnToFace(dir);
  }
}

bool ArcTurn::isFinished() {
  double currentYaw = mouse->getGlobalPose().yaw;
  dYaw = smartmouse::math::yaw_diff(currentYaw, goalYaw);
  return ((fabs(dYaw) < smartmouse::kc::ROT_TOLERANCE) && ((mouse->getCol() != startCol) || (mouse->getRow() != startRow)));
}

void ArcTurn::end() {
  //digitalWrite(RealMouse::LED_3, 1);
  mouse->internalTurnToFace(dir);
  mouse->kinematic_controller.enable_sensor_pose_estimate = true;
}

double ArcTurn::pose_dist(GlobalPose pose, double x, double y) {
  return sqrtf(powf(pose.col-x,2) + powf(pose.row-y,2));
}
