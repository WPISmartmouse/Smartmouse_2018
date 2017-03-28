#pragma once

#include <common/commanduino/CommanDuino.h>
#include <common/Solver.h>

class StepSolveCommand : public CommandGroup {
public:
  StepSolveCommand(Solver *solver);
};
