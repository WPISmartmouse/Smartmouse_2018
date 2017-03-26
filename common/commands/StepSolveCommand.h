#pragma once

#include <common/commanduino/CommandGroup.h>
#include <common/Solver.h>

class StepSolveCommand : public CommandGroup {
public:
  StepSolveCommand(Solver *solver);
};
