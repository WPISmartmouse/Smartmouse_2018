#pragma once

#include "CommanDuino.h"
#include "Flood.h"

class SolveMaze : public CommandGroup {
  public:
    SolveMaze(KnownMaze *maze);
    void initialize();
    bool isFinished();
    void end();

  private:
    KnownMaze *maze;
    Flood solver;

};
