#pragma once

#include "CommanDuino.h"
#include "Solver.h"
#include "KnownMaze.h"

class SolveMaze : public CommandGroup {
  public:
    SolveMaze(Solver *solver);
    void initialize();
    bool isFinished();
    void end();

  private:
    KnownMaze *kmaze;
    Solver *solver;

};
