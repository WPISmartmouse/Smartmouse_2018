#pragma once

#include "CommanDuino.h"
#include "Solver.h"
#include "Mouse.h"

class SpeedRun: public CommandGroup {
  public:
    SpeedRun(Mouse *mouse);
    void initialize();
    bool isFinished();

  private:
    Mouse *mouse;
    char *path;
    int index;
};
