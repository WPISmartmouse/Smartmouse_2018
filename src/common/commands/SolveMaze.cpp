#include "SolveMaze.h"
#include "Forward.h"
#include "Turn.h"
#include "WaitForStart.h"

SolveMaze::SolveMaze(Solver *solver) : CommandGroup("solve"), solver(solver) {
  this->kmaze = solver->kmaze;
}

void SolveMaze::initialize(){
  solver->setup();
}

bool SolveMaze::isFinished(){
  bool groupFinished = CommandGroup::isFinished();

  if (groupFinished) {
    bool mazeSolved = solver->isFinished();

    if (!mazeSolved){
      addSequential(new Turn(kmaze->mouse, solver->planNextStep()));
      addSequential(new Forward(kmaze->mouse));
    }
    else {
      return true;
    }

    return false;
  }

  return false;
}

void SolveMaze::end(){
  solver->teardown();
}
