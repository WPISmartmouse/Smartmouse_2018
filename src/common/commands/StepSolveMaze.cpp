#include "StepSolveMaze.h"
#include "Forward.h"
#include "TurnToFace.h"

StepSolveMaze::StepSolveMaze(KnownMaze *kmaze, Solver *solver) : CommandGroup("step_solve_maze"), kmaze(kmaze), solver(solver) {
}

void StepSolveMaze::initialize(){
  addSequential(new TurnToFace(kmaze->mouse, solver->planNextStep()));
  addSequential(new Forward(kmaze->mouse));
}

bool StepSolveMaze::isFinished(){
  return solver->isFinished();
}
