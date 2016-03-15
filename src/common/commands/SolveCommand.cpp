#include "SolveCommand.h"
#include "WaitForStart.h"
#include "SolveMaze.h"
#include "WallFollow.h"
#include "Flood.h"
#include "Finish.h"

SolveCommand::SolveCommand(Solver *solver) : CommandGroup("Solve") {
  addSequential(new WaitForStart());
  addSequential(new SolveMaze(solver));
  //addSequential(new ReturnToStart());
  //addSequential(new SpeedRun());
  addSequential(new Finish(solver->mouse->maze));
}
