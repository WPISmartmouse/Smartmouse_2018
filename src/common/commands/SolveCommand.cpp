#include "SolveCommand.h"
#include "WaitForStart.h"
#include "SolveMaze.h"

SolveCommand::SolveCommand(KnownMaze *maze) : CommandGroup("Solve") {
  addSequential(new WaitForStart());
  addSequential(new SolveMaze(maze));
  //addSequential(new ReturnToStart());
  //addSequential(new SpeedRun());

}
