#include "StepSolveMaze.h"
#include "Forward.h"
#include "Turn.h"
#include "WaitForStart.h"

StepSolveMaze::StepSolveMaze(Solver *solver) : CommandGroup("step_solve"), solver(solver) {
  this->kmaze = solver->kmaze;
}

void StepSolveMaze::initialize(){
  solver->setup();
}

void StepSolveMaze::execute(){
  CommandGroup::execute();
}

bool StepSolveMaze::isFinished(){
  bool groupFinished = CommandGroup::isFinished();

  if (groupFinished) {
    bool mazeSolved = solver->isFinished();

    if (!mazeSolved){
      addSequential(new Turn(kmaze->mouse, solver->planNextStep()));
      addSequential(new Forward(kmaze->mouse));
      addSequential(new WaitForStart());
      kmaze->print_maze_mouse();
    }
    else {
      return true;
    }

    return false;
  }

  return false;
}

void StepSolveMaze::end(){
  solver->teardown();
}
