#pragma once

#include <common/commanduino/CommandGroup.h>
#include <common/Solver.h>

class SolveCommand : public CommandGroup {
  public:
    SolveCommand(Solver *solver);
};
