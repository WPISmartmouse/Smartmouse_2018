#pragma once
#include "CommanDuino.h"
#include "Solver.h"

class SolveCommand : public CommandGroup {
  public:
    SolveCommand(Solver *solver);
};
