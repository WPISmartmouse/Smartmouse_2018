#include "SolveCommand.h"
#include "WaitForStart.h"
#include "SolveMaze.h"
#include "ReturnToStart.h"
#include "WallFollow.h"
#include "SpeedRun.h"
#include "Calibration.h"
#include "Flood.h"
#include "Finish.h"

SolveCommand::SolveCommand(Solver *solver) : CommandGroup("Solve") {
  addSequential(new Calibration());
  addSequential(new WaitForStart());
  addSequential(new SolveMaze(solver));
  //addSequential(new ReturnToStart(solver->mouse));
  //addSequential(new SpeedRun(solver->mouse));
  //addSequential(new ReturnToStart(solver->mouse));
  //addSequential(new Finish(solver->mouse->maze));
}
