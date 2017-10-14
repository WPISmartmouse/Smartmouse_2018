#include "SolveMaze.h"
#include "Forward.h"
#include "ForwardToCenter.h"
#include "Turn.h"
#include "TurnInPlace.h"

SolveMaze::SolveMaze(Solver *solver, Solver::Goal goal) : CommandGroup("solve"), solver(solver), movements(0),
                                                          goal(goal) {}

void SolveMaze::initialize() {
  solved = false;
  atCenter = false;
  solver->setGoal(goal);
}

bool SolveMaze::isFinished() {
  bool groupFinished = CommandGroup::isFinished();

  if (groupFinished) {
    bool mazeSolved = solver->isFinished();

    if (!mazeSolved) {
      motion_primitive_t prim = solver->planNextStep();

      if (!solver->isSolvable()) {
        solved = false;
        return true;
      }

      if (prim.d == solver->mouse->getDir()) {
        for (size_t i = 0; i < prim.n; i++) {
          addSequential(new Forward());
        }
      } else {
        addSequential(new Turn(prim.d));
      }

      movements++;
    } else if (!atCenter){
      addSequential(new ForwardToCenter());
      if (goal == Solver::Goal::START) {
        addSequential(new TurnInPlace(Direction::E));
      }
      atCenter = true;
      solved = true;
      return false;
    } else {
      return true;
    }

    return false;
  }

  return false;
}

void SolveMaze::end() {
  solver->teardown();
}
