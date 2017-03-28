#pragma once

#include <common/commanduino/CommanDuino.h>
#include <common/Solver.h>

class SolveCommand : public CommandGroup {
public:
  SolveCommand(Solver *solver);
};
