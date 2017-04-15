#include "Solver.h"

Solver::Solver(Mouse *mouse) : mouse(mouse), solvable(true) {}

bool Solver::isSolvable() {
  return solvable;
}
