#pragma once


#include <common/commanduino/CommandGroup.h>
#include <common/Solver.h>

class SolveMaze : public CommandGroup {
  public:
    SolveMaze(Solver *solver);
    void initialize();
    bool isFinished();
    void end();

  private:
    Solver *solver;

};
