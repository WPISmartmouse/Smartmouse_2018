#include "StepSolveCommand.h"
#include "WaitForStart.h"
#include "StepSolveMaze.h"
#include "WallFollow.h"
#include "Flood.h"
#include "Finish.h"

StepSolveCommand::StepSolveCommand(KnownMaze *maze) : CommandGroup("Solve") {
  addSequential(new WaitForStart());
  addSequential(new StepSolveMaze(new WallFollow(maze)));
  //addSequential(new ReturnToStart());
  //addSequential(new SpeedRun());
  addSequential(new Finish());

}
