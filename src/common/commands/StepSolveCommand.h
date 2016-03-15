#pragma once
#include "CommanDuino.h"
#include "Solver.h"

class StepSolveCommand : public CommandGroup {
  public:
    StepSolveCommand(Solver *solver);
};
