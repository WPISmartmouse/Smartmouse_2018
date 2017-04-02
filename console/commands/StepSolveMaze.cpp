#include "StepSolveMaze.h"
#include "Forward.h"
#include "Turn.h"
#include "WaitForStart.h"

StepSolveMaze::StepSolveMaze(Solver *solver) : CommandGroup("step_solve"), solver(solver) {}

void StepSolveMaze::initialize() {
  solver->setup();
}

void StepSolveMaze::execute() {
  CommandGroup::execute();
}

bool StepSolveMaze::isFinished() {
  bool groupFinished = CommandGroup::isFinished();

  if (groupFinished) {
    bool mazeSolved = solver->isFinished();

    if (!mazeSolved) {
      addSequential(new Turn(solver->planNextStep()));
      addSequential(new Forward());
      addSequential(new WaitForStart());
#ifndef EMBED
      solver->mouse->print_maze_mouse();
#endif
    } else {
      return true;
    }

    return false;
  }

  return false;
}

void StepSolveMaze::end() {
  solver->teardown();
}
