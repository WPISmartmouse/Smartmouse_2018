#include "SolveMaze.h"
#include "Forward.h"
#include "Turn.h"

SolveMaze::SolveMaze(Solver *solver) : CommandGroup("solve"), solver(solver) {}

void SolveMaze::initialize() {
  solver->setup();
}

bool SolveMaze::isFinished() {
  bool groupFinished = CommandGroup::isFinished();

  if (groupFinished) {
    bool mazeSolved = solver->isFinished();

    if (!mazeSolved) {
      Direction next_direction = solver->planNextStep();
      printf("Next Direction: %c\n", dir_to_char(next_direction));
      addSequential(new Turn(next_direction));
      addSequential(new Forward());
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
