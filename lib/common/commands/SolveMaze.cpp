#include "SolveMaze.h"
#include "Forward.h"
#include "Turn.h"
#include "WaitForStart.h"

SolveMaze::SolveMaze(Solver *solver) : CommandGroup("solve"), solver(solver) { }

void SolveMaze::initialize(){
  solver->setup();
}

bool SolveMaze::isFinished(){
  bool groupFinished = CommandGroup::isFinished();

  if (groupFinished) {
    bool mazeSolved = solver->isFinished();

    if (!mazeSolved){
      addSequential(new Turn(solver->planNextStep()));
      addSequential(new Forward());
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
