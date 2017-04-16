#include "WallFollow.h"

#ifndef EMBED

#endif

WallFollow::WallFollow(Mouse *mouse) : Solver(mouse) {}

void WallFollow::setup() {
  mouse->reset();
  mouse->maze->reset();
  mouse->maze->mark_origin_known();
  goal_row = AbstractMaze::MAZE_SIZE/2;
  goal_col = AbstractMaze::MAZE_SIZE/2;
  step = 0;
}

void WallFollow::setGoal(unsigned int row, unsigned int col) {
  goal_row = row;
  goal_col = col;
}

char *WallFollow::solve() {
  //run till you find the goal
  while (!isFinished()) {
    Direction nextDir = planNextStep();
    if (nextDir != Direction::INVALID) {
      mouse->internalTurnToFace(nextDir);
      mouse->internalForward();
    }
  }
  teardown();
  return mouse->maze->fastest_route;
}

bool WallFollow::isFinished() {
  return !solvable || (mouse->getRow() == goal_row && mouse->getCol() == goal_col);
}

void WallFollow::teardown() {
  mouse->maze->fastest_route[step] = 0;
}

Direction WallFollow::planNextStep() {
  Direction dir = left_of_dir(mouse->getDir());
  Direction nextDir;

  SensorReading sr = mouse->checkWalls();

  if (!sr.isWall(dir)) {
    //if you can turn left you must
    nextDir = dir;
  } else if (!sr.isWall(mouse->getDir())) {
    nextDir = mouse->getDir();
  } else if (!sr.isWall(opposite_direction(dir))) {
    //if you can't go left or forward try right
    nextDir = opposite_direction(dir);
  } else if (!sr.isWall(opposite_direction(mouse->getDir()))){
    //you must do a 180
    nextDir = opposite_direction(mouse->getDir());
  }
  else {
    solvable = false;
    return Direction::INVALID;
  }

  mouse->maze->fastest_route[step++] = dir_to_char(nextDir);
  mouse->mark_mouse_position_visited();

  return nextDir;
}
