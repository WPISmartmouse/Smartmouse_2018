#include "SolveMaze.h"
#include "StepSolveMaze.h"

SolveMaze::SolveMaze(KnownMaze *maze) : CommandGroup("solve"), solver(maze) {
  this->maze = maze;
}

void SolveMaze::initialize(){
  solver.setup();
  addSequential(new StepSolveMaze(maze, &solver));
}

bool SolveMaze::isFinished(){
  bool finished = solver.isFinished();

  if (!finished){
    addSequential(new StepSolveMaze(maze, &solver));
  }

  return finished;
}

void SolveMaze::end(){
  solver.teardown();
}
