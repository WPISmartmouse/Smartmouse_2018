#include "SolveCommand.h"
#include "WaitForStart.h"
#include "SolveMaze.h"

SolveCommand::SolveCommand() : CommandGroup("Solve") {
  addSequential(new WaitForStart());
  addSequential(new SolveMaze());
  //addSequential(new ReturnToStart());
  //addSequential(new SpeedRun());

}
