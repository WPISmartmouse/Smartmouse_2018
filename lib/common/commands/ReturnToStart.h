#pragma once

#include "CommanDuino.h"
#include "Solver.h"
#include "Mouse.h"

class ReturnToStart: public CommandGroup {
  public:
    ReturnToStart(Mouse *mouse);
    void initialize();
    bool isFinished();

  private:
    char *pathToStart;
    int index;
    Mouse *mouse;

};
