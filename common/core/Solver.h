#pragma once

#include "Mouse.h"


class Solver {
public:
  enum class Goal {
    CENTER,
    START
  };

  Solver(Mouse *mouse);

  virtual void setup() = 0;

  virtual motion_primitive_t planNextStep() = 0;

  virtual bool isFinished() = 0;

  virtual route_t solve() = 0;

  virtual void teardown() = 0;

  virtual void setGoal(Goal goal) = 0;

  bool isSolvable();

  bool solvable;
  Mouse *mouse;
};
