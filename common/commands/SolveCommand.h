#pragma once

#include <common/commanduino/CommanDuino.h>
#include <common/Solver.h>
#include "SolveMaze.h"

class SolveCommand : public CommandGroup {
public:
  SolveCommand(Solver *solver);

  void initialize();

  bool isFinished();

private:
  Solver *solver;
  static constexpr int MAX_RUNS = 2;
  SolveMaze *to_center, *to_start;
  int runs;
};
