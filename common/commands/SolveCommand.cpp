#include "SolveCommand.h"
#include "WaitForStart.h"
#include "SolveMaze.h"
#include "Finish.h"
#include "SpeedRun.h"
#include "ReturnToStart.h"

SolveCommand::SolveCommand(Solver *solver) : CommandGroup("SolveGroup") {
  addSequential(new WaitForStart());
  solver->setup();
  addSequential(new SolveMaze(solver));
  addSequential(new SolveMaze(solver, 0, 0));
//  addSequential(new SpeedRun(solver->mouse));
//  addSequential(new ReturnToStart(solver->mouse));
//  addSequential(new Finish(solver->mouse->maze));
}
