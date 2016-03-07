#pragma once
#include "CommanDuino.h"
#include "KnownMaze.h"

class SolveCommand : public CommandGroup {
  public:
    SolveCommand(KnownMaze *maze);
};
