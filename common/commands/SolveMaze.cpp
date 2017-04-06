#include "SolveMaze.h"
#include "Forward.h"
#include "Turn.h"
#include "WaitForStart.h"

SolveMaze::SolveMaze(Solver *solver) : CommandGroup("solve"), solver(solver) {}

void SolveMaze::initialize() {
  solver->setup();
}

bool SolveMaze::isFinished() {
  bool groupFinished = CommandGroup::isFinished();

  if (groupFinished) {
    bool mazeSolved = solver->isFinished();

    if (!mazeSolved) {
      Direction nextDirection = solver->planNextStep();
#ifndef CONSOLE
      if (nextDirection == solver->mouse->getDir()) {
        addSequential(new Forward());
      }
      else {
        addSequential(new Turn(nextDirection));
      }
#else
      addSequential(new Turn(nextDirection));
      addSequential(new Forward());
      addSequential(new WaitForStart());
      solver->mouse->print_maze_mouse();
#endif
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
