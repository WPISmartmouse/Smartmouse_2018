/** functions for exploring and solving mazes */
#pragma once

#include "KnownMaze.h"
#include "Direction.h"
#include "Node.h"
#include "Mouse.h"
#include <stdbool.h>

class Solvers {

  public:
    /** \brief starts at 0,0 and explores the whole maze.
     * kmaze is the known maze, and is used to "read the sensors".
     * The mouse solves the maze after sensing each new square.
     * solves assuming ALL walls, and assuming NO walls.
     * when the solution for those two mazes are the same, then it knows the fastest route.
     * @param kmaze the known maze. in reality this won't exist because the sensors can read from the real world.
     */
    static void flood_explore(KnownMaze kmaze);

    /** \brief starts at 0,0 and explores the whole maze.
     * simply fallows the left hand wall.
     * We know this won't solve the competition maze.
     * @param kmaze the known maze. in reality this won't exist because the sensors can read from the real world
     */
    static void left_hand_follow(KnownMaze maze);
};
