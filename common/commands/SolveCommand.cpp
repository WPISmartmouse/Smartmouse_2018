#include "Stop.h"
#include "SolveCommand.h"
#include "WaitForStart.h"
#include "SolveMaze.h"
#include "Finish.h"
#include "SpeedRun.h"
#include "ReturnToStart.h"

SolveCommand::SolveCommand(Solver *solver) : CommandGroup("SolveGroup") {
  addSequential(new WaitForStart());
  solver->setup();

  //repeat these two as many times as you want
  addSequential(new SolveMaze(solver));
  addSequential(new SolveMaze(solver, 0, 0));

  addSequential(new SolveMaze(solver));
  addSequential(new SolveMaze(solver, 0, 0));

  addSequential(new Stop(200));
}
