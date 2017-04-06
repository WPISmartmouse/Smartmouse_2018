#include "SolveMaze.h"
#include "Forward.h"
#include "Turn.h"
#include "WaitForStart.h"

SolveMaze::SolveMaze(Solver *solver) : CommandGroup("solve"), solver(solver), movements(0) {}

void SolveMaze::initialize() {
  solver->setup();
}

bool SolveMaze::isFinished() {
  bool groupFinished = CommandGroup::isFinished();

  if (groupFinished) {
    bool mazeSolved = solver->isFinished();

    if (!mazeSolved) {
      Direction nextDirection = solver->planNextStep();
      if (nextDirection == solver->mouse->getDir()) {
        addSequential(new Forward());
      }
      else {
        addSequential(new Turn(nextDirection));
      }

      movements++;

      Direction left = left_of_dir(solver->mouse->getDir());
      Direction right = right_of_dir(solver->mouse->getDir());
      if (movements > 10 && solver->mouse->isWallInDirection(left) && solver->mouse->isWallInDirection(right)) {
        printf("Scheduling Re-Align!");
//        addSequential(new AlignYaw());
      }
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
