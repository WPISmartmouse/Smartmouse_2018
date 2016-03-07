#pragma once

#include "CommanDuino.h"
#include "KnownMaze.h"
#include "Solver.h"

class StepSolveMaze : public CommandGroup {
  public:
    StepSolveMaze(KnownMaze *maze, Solver *solver);
    void initialize();
    bool isFinished();

  private:
    Solver *solver;
    KnownMaze *kmaze;
};
