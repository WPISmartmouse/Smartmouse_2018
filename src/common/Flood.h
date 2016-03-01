/** \brief starts at 0,0 and explores the whole maze.
 * kmaze is the known maze, and is used to "read the sensors".
 * The mouse solves the maze after sensing each new square.
 * solves assuming ALL walls, and assuming NO walls.
 * when the solution for those two mazes are the same,
 * then it knows the fastest route.
 */
#pragma once

#include "KnownMaze.h"
#include "Solver.h"

class Flood : public Solver {

  public:

    Flood();

    void setup(KnownMaze *kmaze);
    AbstractMaze stepOnce();
    char *solve();
    void teardown();
    bool isFinished();

  private:
    KnownMaze *kmaze;

    /// \brief this maze is initially no walls,  and walls are filled out every time the mouse moves
    AbstractMaze no_wall_maze;

    /// \brief this maze is initially no walls,  and walls are removed every time the mouse moves
    AbstractMaze all_wall_maze;

    char *no_wall_path;
    char *all_wall_path;
    char *final_solution;

    bool solvable, solved, done;
};
