#include "SolveMaze.h"
#include "Forward.h"
#include "Turn.h"

SolveMaze::SolveMaze(Solver *solver, Solver::Goal goal) : CommandGroup("solve"), solver(solver), movements(0),
                                                          goal(goal) {}

void SolveMaze::initialize() {
  solved = false;
  solver->setGoal(goal);
}

bool SolveMaze::isFinished() {
  bool groupFinished = CommandGroup::isFinished();

  if (groupFinished) {
    bool mazeSolved = solver->isFinished();

    if (!mazeSolved) {
      Direction nextDirection = solver->planNextStep();

      if (!solver->isSolvable()) {
        solved = false;
        return true;
      }

      if (nextDirection == solver->mouse->getDir()) {
        addSequential(new Forward());
      } else {
        addSequential(new Turn(nextDirection));
      }

      movements++;
    } else {
      solved = true;
      return true;
    }

    return false;
  }

  return false;
}

void SolveMaze::end() {
  solver->teardown();
}
