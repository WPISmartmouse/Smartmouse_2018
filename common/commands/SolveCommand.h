#pragma once

#include <common/commanduino/CommanDuino.h>
#include <common/Solver.h>

class SolveCommand : public CommandGroup {
public:
  SolveCommand(Solver *solver);

  void initialize();
  bool isFinished();

private:
  Solver *solver;
  static constexpr int MAX_RUNS = 4;
  int runs;
};
