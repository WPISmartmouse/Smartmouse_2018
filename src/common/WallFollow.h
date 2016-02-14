/** \brief starts at 0,0 and explores the whole maze.
 * simply fallows the left hand wall.
 * We know this won't solve the competition maze.
 */
#pragma once

#include "Solver.h"
#include "KnownMaze.h"

class WallFollow : public Solver {

  public:

    void setup(KnownMaze kmaze);
    AbstractMaze stepOnce();
    char *solve();
    void teardown();
    bool isFinished();

  private:
    KnownMaze kmaze;
    int step;
};
