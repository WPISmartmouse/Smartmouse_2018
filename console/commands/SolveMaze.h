#pragma once

#include "CommanDuino.h"
#include "Solver.h"
#include "Mouse.h"

class SolveMaze : public CommandGroup {
  public:
    SolveMaze(Solver *solver);
    void initialize();
    bool isFinished();
    void end();

  private:
    Solver *solver;

};
