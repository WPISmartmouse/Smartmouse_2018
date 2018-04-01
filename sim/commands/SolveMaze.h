#pragma once

#include <common/commanduino/CommanDuino.h>
#include <common/core/Solver.h>
#include <common/core/Mouse.h>

class SolveMaze : public CommandGroup {
public:
  SolveMaze(Solver *solver, Solver::Goal goal);

  void initialize();

  bool isFinished();

  void end();

private:
  Solver *solver;
  int movements;
  Solver::Goal goal;
  bool solved;
  bool at_center;

};
