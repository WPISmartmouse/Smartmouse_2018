#pragma once

#include <common/commanduino/CommanDuino.h>
#include <common/Solver.h>
#include <common/Mouse.h>

class SolveMaze : public CommandGroup {
public:
  SolveMaze(Solver *solver);
  SolveMaze(Solver *solver, unsigned int goal_row, unsigned int goal_col);

  void initialize();

  bool isFinished();

  void end();

private:
  Solver *solver;
  int movements;
  unsigned int goal_row, goal_col;

};
