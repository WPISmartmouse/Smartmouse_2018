#include "StepSolveCommand.h"
#include "WaitForStart.h"
#include "StepSolveMaze.h"
#include "WallFollow.h"
#include "Flood.h"

class End : public Command {
#include <stdio.h>
  public: End() : Command("end"){}
  public: void initialize(){printf("done\n.");}
  public: bool isFinished(){return true;}
};

StepSolveCommand::StepSolveCommand(KnownMaze *maze) : CommandGroup("Solve") {
  addSequential(new WaitForStart());
  addSequential(new StepSolveMaze(new WallFollow(maze)));
  addSequential(new End());
  //addSequential(new ReturnToStart());
  //addSequential(new SpeedRun());

}
