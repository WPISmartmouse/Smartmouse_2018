#pragma once

#include "AbstractMaze.h"
#include "KnownMaze.h"
#include "Mouse.h"

class Solver {
  public:
    Solver(KnownMaze *maze);
    virtual void setup() = 0;
    virtual AbstractMaze stepOnce() = 0;
    virtual bool isFinished() = 0;
    virtual char *solve() = 0;
    virtual void teardown() = 0;

  protected:
    KnownMaze *kmaze;
};
