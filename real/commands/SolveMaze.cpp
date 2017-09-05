#include <commands/SolveMaze.h>
#include <commands/Forward.h>
#include <commands/Turn.h>
#include <commands/ForwardToCenter.h>
#include <commands/TurnInPlace.h>

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
  print("%lu\r\n", getTime());
  solver->teardown();
}
