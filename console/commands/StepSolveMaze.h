#pragma once

#include <common/commanduino/CommanDuino.h>
#include <common/Solver.h>
#include <common/Mouse.h>

class StepSolveMaze : public CommandGroup {
public:
  StepSolveMaze(Solver *solver);

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:
  Solver *solver;
};
