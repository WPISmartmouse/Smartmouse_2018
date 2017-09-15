#include "Solver.h"

Solver::Solver(Mouse *mouse) : solvable(true), mouse(mouse) {}

bool Solver::isSolvable() {
  return solvable;
}
