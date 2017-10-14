/** \brief starts at 0,0 and explores the whole maze.
 * kmaze is the known maze, and is used to "read the sensors".
 * The mouse solves the maze after sensing each new square.
 * solves assuming ALL walls, and assuming NO walls.
 * when the solution for those two mazes are the same,
 * then it knows the fastest route.
 */
#pragma once

#include "Solver.h"
#include "Mouse.h"

class Flood : public Solver {

public:

  Flood(Mouse *mouse);

  virtual void setup() override;

  virtual motion_primitive_t planNextStep() override;

  virtual route_t solve() override;

  virtual void teardown() override;

  virtual bool isFinished() override;

  virtual void setGoal(Solver::Goal goal) override;

  bool done;

private:

  /// \brief this maze is initially no walls, and walls are filled out every time the mouse moves
  AbstractMaze no_wall_maze;

  /// \brief this maze is initially all walls, and walls are removed every time the mouse moves
  AbstractMaze *all_wall_maze;

  route_t no_wall_path;
  route_t all_wall_path;
  Solver::Goal goal;

  bool solved;
};
