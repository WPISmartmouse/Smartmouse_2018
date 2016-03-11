#pragma once

#include "AbstractMaze.h"
#include "KnownMaze.h"
#include "Mouse.h"

class Solver {
  public:
    Solver(KnownMaze *maze);
    virtual void setup() = 0;
    virtual Direction planNextStep() = 0;
    virtual bool isFinished() = 0;
    virtual char *solve() = 0;
    virtual void teardown() = 0;
    KnownMaze *kmaze;
};
