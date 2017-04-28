
#include "ArcTurn.h"

ArcTurn::ArcTurn(Direction dir) : Command("RealArcTurn"), mouse(RealMouse::inst()), dir(dir) {}

void ArcTurn::initialize() {
  mouse->kinematic_controller.ignore_sensor_pose_estimate = true;
  setTimeout(2000);
  goalYaw = dir_to_yaw(dir);
  startPose = mouse->getPose();

  // determine vtc point of arc
  // inital center of sqaure

  startCol = mouse->getCol();
  startRow = mouse->getRow();
  vtc_x = mouse->getCol()*AbstractMaze::UNIT_DIST + AbstractMaze::HALF_UNIT_DIST;
  vtc_y = mouse->getRow()*AbstractMaze::UNIT_DIST + AbstractMaze::HALF_UNIT_DIST;

  end_x = vtc_x;
  end_y = vtc_y;

  // move to corner
  switch (mouse->getDir()) {
    case Direction::N: {
      vtc_y += AbstractMaze::HALF_UNIT_DIST;
      if (dir == Direction::E){
        vtc_x += AbstractMaze::HALF_UNIT_DIST;
      } else {
        vtc_x -= AbstractMaze::HALF_UNIT_DIST;
      }
      end_x = vtc_x;
      break;
    }
    case Direction::E: {
      vtc_x -= AbstractMaze::HALF_UNIT_DIST;
      if (dir == Direction::S) {
        vtc_y += AbstractMaze::HALF_UNIT_DIST;
      } else {
        vtc_y -= AbstractMaze::HALF_UNIT_DIST;
      }
      end_y = vtc_y;
      break;
    }
    case Direction::S: {
      vtc_y -= AbstractMaze::HALF_UNIT_DIST;
      if (dir == Direction::W){
        vtc_x -= AbstractMaze::HALF_UNIT_DIST;
      } else {
        vtc_x += AbstractMaze::HALF_UNIT_DIST;
      }
      end_x = vtc_x;
      break;
    }
    case Direction::W: {
      vtc_x += AbstractMaze::HALF_UNIT_DIST;
      if (dir == Direction::N){
        vtc_y -= AbstractMaze::HALF_UNIT_DIST;
      } else {
        vtc_y += AbstractMaze::HALF_UNIT_DIST;
      }
      end_y = vtc_y;
      break;
    }
    default:
      exit(0);
  }

  //print("%f,%f\n\r", vtc_x, vtc_y);

  mouse->kinematic_controller.ignore_sensor_pose_estimate = true;
  // when we get close to aligned, there might be a wall we can use to better estimate our angle

}

void ArcTurn::execute() {

  Pose curPose = mouse->getPose();
  double cur_x = fabs(curPose.x - vtc_x);
  double cur_y = fabs(curPose.y - vtc_y);

  double dAngle;
  if ((dir == Direction::N)||(dir == Direction::S)) {
    dAngle = atanf(fabs(cur_x/cur_y));
  } else {
    dAngle = atanf(fabs(cur_y/cur_x));
  }

  double ang_error = fabs(mouse->kinematic_controller.yawDiff(curPose.yaw, dir_to_yaw(mouse->getDir())))-dAngle;
  double arc_error = (AbstractMaze::HALF_UNIT_DIST/pose_dist(mouse->getPose(), vtc_x, vtc_y))-1;

  double corr = (ang_error*1.0)+(arc_error*0.25);
  //print("%f,%f,%f\n\r", dAngle, cur_x, cur_y);//mouse->kinematic_controller.yawDiff(curPose.yaw, dir_to_yaw(mouse->getDir())));

  double fast_speed = FAST_ARC_SPEED*((corr*-3)+1);
  double slow_speed = SLOW_ARC_SPEED*((corr*3)+1);

  if (right_of_dir(mouse->getDir()) == dir ){
    mouse->setSpeed(fast_speed, slow_speed);
  } else {
    mouse->setSpeed(slow_speed, fast_speed);
  }
  // this allows us to use that
  if (fabs(dYaw) < 0.1 && mouse->kinematic_controller.ignore_sensor_pose_estimate) {
    //digitalWrite(RealMouse::LED_2, 1);
    // FIXME: this is kind of a hack. It's needed because WallFollower checks dir in order to compute
    // FIXME: the correct yaw. it adds dir_to_yaw(getDir()), so we must assume we're close enough
    //mouse->kinematic_controller.ignore_sensor_pose_estimate = false;
    //mouse->internalTurnToFace(dir);
  }
}

bool ArcTurn::isFinished() {
  double currentYaw = mouse->getPose().yaw;
  dYaw = WallFollower::yawDiff(currentYaw, goalYaw);
  return ((fabs(dYaw) < config.ROT_TOLERANCE) && ((mouse->getCol() != startCol) || (mouse->getRow() != startRow)));
}

void ArcTurn::end() {
  //digitalWrite(RealMouse::LED_3, 1);
  mouse->internalTurnToFace(dir);
  mouse->kinematic_controller.ignore_sensor_pose_estimate = false;
}

double ArcTurn::pose_dist(Pose pose, double x, double y) {
  return sqrtf(powf(pose.x-x,2) + powf(pose.y-y,2));
}