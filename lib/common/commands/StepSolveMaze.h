#pragma once

#include "CommanDuino.h"
#include "Solver.h"
#include "Mouse.h"

class StepSolveMaze : public CommandGroup {
  public:
    StepSolveMaze(Solver *solver);
    void initialize();
    void execute();
    bool isFinished();
    void end();

  private:
    Solver *solver;
};
