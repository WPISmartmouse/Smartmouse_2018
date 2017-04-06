#include <common/RobotConfig.h>
#include "ArcTurn.h"

const double ArcTurn::kTurn = 4.5;

ArcTurn::ArcTurn(Direction dir) : Command("SimArcTurn"), mouse(SimMouse::inst()), dir(dir) {}

void ArcTurn::initialize() {
  left = (dir == left_of_dir(mouse->getDir()));
  // if we're further into the square, turn harder
  double disp_from_last_edge = AbstractMaze::UNIT_DIST - WallFollower::dispToNextEdge(mouse);
  turn_effort = disp_from_last_edge * kTurn;
  start_row = mouse->getRow();
  start_col = mouse->getCol();
  print("arc turn\n");
}

void ArcTurn::execute() {
  double goalYaw = dir_to_yaw(dir);
  double currentYaw = mouse->getPose().yaw;
  double dYaw = WallFollower::yawDiff(currentYaw, goalYaw);

  if (fabs(dYaw) < Mouse::ROT_TOLERANCE) {
    mouse->setSpeed(0.07, 0.07);
  }
  else if (left) {
    mouse->setSpeed(0.0, std::max(0.07 + turn_effort, 0.12));
  } else {
    mouse->setSpeed(std::max(0.07 + turn_effort, 0.12), 0.0);
  }
}

bool ArcTurn::isFinished() {
  double goalYaw = dir_to_yaw(dir);
  Pose current_pose = mouse->getPose();
  double currentYaw = current_pose.yaw;
  double dYaw = WallFollower::yawDiff(currentYaw, goalYaw);
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

  print("%f\n", edgeDisp);
  return (fabs(dYaw) < Mouse::ROT_TOLERANCE) && (edgeDisp <= 0);
}

void ArcTurn::end() {
  print("turning done.\n");
  mouse->internalTurnToFace(dir);
}
