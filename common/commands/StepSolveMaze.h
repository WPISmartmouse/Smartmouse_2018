#pragma once


#include <common/commanduino/CommandGroup.h>
#include <common/Solver.h>

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
