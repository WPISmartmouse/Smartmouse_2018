#pragma once

#include <common/commanduino/CommanDuino.h>
#include <common/Solver.h>
#include <common/Mouse.h>

class SolveMaze : public CommandGroup {
public:
  SolveMaze(Solver *solver);
  SolveMaze(Solver *solver, int goal_row, int goal_col);

  void initialize();

  bool isFinished();

  void end();

private:
  Solver *solver;
  int movements, goal_row, goal_col;

  bool solved;
};
