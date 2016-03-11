#pragma once
#include "CommanDuino.h"
#include "KnownMaze.h"

class StepSolveCommand : public CommandGroup {
  public:
    StepSolveCommand(KnownMaze *maze);
};
