#include <common/RobotConfig.h>
#include "ArcTurn.h"

const double ArcTurn::kTurn = 9.5;

ArcTurn::ArcTurn(Direction dir) : Command("SimArcTurn"), mouse(SimMouse::inst()), dir(dir) {}

void ArcTurn::initialize() {
  mouse->ignore_sensor_pose_estimate = true;
  left = (dir == left_of_dir(mouse->getDir()));
  // if we're further into the square, turn harder
  double disp_from_last_edge = AbstractMaze::UNIT_DIST - WallFollower::dispToNextEdge(mouse);
  turn_effort = disp_from_last_edge * kTurn;
  start_row = mouse->getRow();
  start_col = mouse->getCol();
}

void ArcTurn::execute() {
  // when we get close to aligned, there might be a wall we can use to better estimate our angle
  // this allows us to use that
  if (fabs(dYaw) < 0.1 && mouse->ignore_sensor_pose_estimate) {
    mouse->ignore_sensor_pose_estimate = false;
    // FIXME: this is kind of a hack. It's needed because WallFollower checks dir in order to compute
    // FIXME: the correct yaw. it adds dir_to_yaw(getDir()), so we must assume we're close enough
    mouse->internalTurnToFace(dir);
  }

  double goalYaw = dir_to_yaw(dir);
  double currentYaw = mouse->getPose().yaw;
  double dYaw = WallFollower::yawDiff(currentYaw, goalYaw);


  if (fabs(dYaw) < Mouse::ROT_TOLERANCE) {
    mouse->setSpeed(0.07, 0.07);
  }
  else if (left) {
    mouse->setSpeed(0.0, std::max(0.07 + turn_effort, SimMouse::CONFIG.MAX_SPEED));
  } else {
    mouse->setSpeed(std::max(0.07 + turn_effort, SimMouse::CONFIG.MAX_SPEED), 0.0);
  }
}

bool ArcTurn::isFinished() {
  double goalYaw = dir_to_yaw(dir);
  Pose current_pose = mouse->getPose();
  double currentYaw = current_pose.yaw;
  dYaw = WallFollower::yawDiff(currentYaw, goalYaw);
  double edgeDisp;

  switch (dir) {
    case Direction::N: {
      double next_row_y = start_row * AbstractMaze::UNIT_DIST;
      edgeDisp = current_pose.y - next_row_y;
      break;
    }
    case Direction::S: {
      double next_row_y = (start_row + 1) * AbstractMaze::UNIT_DIST;
      edgeDisp = next_row_y - current_pose.y;
      break;
    }
    case Direction::E: {
      double next_col_x = (start_col + 1) * AbstractMaze::UNIT_DIST;
      edgeDisp =  next_col_x - current_pose.x;
      break;
    }
    case Direction::W: {
      double next_col_x = start_col * AbstractMaze::UNIT_DIST;
      edgeDisp =  current_pose.x - next_col_x;
      break;
    }
  }

  return (fabs(dYaw) < Mouse::ROT_TOLERANCE) && (edgeDisp <= 0);
}

void ArcTurn::end() {
  print("turning done.\n");
  mouse->internalTurnToFace(dir);
}
