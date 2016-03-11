#include "SolveCommand.h"
#include "WaitForStart.h"
#include "SolveMaze.h"
#include "WallFollow.h"
#include "Flood.h"

SolveCommand::SolveCommand(KnownMaze *maze) : CommandGroup("Solve") {
  addSequential(new WaitForStart());
  addSequential(new SolveMaze(new WallFollow(maze)));
  //addSequential(new ReturnToStart());
  //addSequential(new SpeedRun());

}
