#pragma once

#include "CommanDuino.h"
#include "KnownMaze.h"
#include "Solver.h"

class StepSolveMaze : public CommandGroup {
  public:
    StepSolveMaze(Solver *solver);
    void initialize();
    void execute();
    bool isFinished();
    void end();

  private:
    Solver *solver;
    KnownMaze *kmaze;
};
