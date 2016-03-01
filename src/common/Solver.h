#pragma once

#include "AbstractMaze.h"
#include "KnownMaze.h"

class Solver {
  public:
    virtual void setup(KnownMaze *kmaze) = 0;
    virtual AbstractMaze stepOnce() = 0;
    virtual bool isFinished() = 0;
    virtual char *solve() = 0;
    virtual void teardown() = 0;
};
