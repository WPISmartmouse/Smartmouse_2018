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
      addSequential(new Turn(nextDirection));
      addSequential(new Forward());
      addSequential(new WaitForStart());
      solver->mouse->print_maze_mouse();
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
