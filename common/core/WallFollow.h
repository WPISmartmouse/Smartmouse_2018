/** \brief starts at 0,0 and explores the whole maze.
 * simply fallows the left hand wall.
 * We know this won't solve the competition maze.
 */
#pragma once

#include "Solver.h"
#include "Mouse.h"

class WallFollow : public Solver {

public:

  WallFollow(Mouse *mouse);

  virtual void setup() override;

  virtual motion_primitive_t planNextStep() override;

  virtual route_t solve() override;

  virtual void teardown() override;

  virtual bool isFinished() override;

  virtual void setGoal(Solver::Goal goal) override;

private:
  Solver::Goal goal;
};
