#include <common/commands/SpeedRun.h>
#include <common/commands/ReturnToStart.h>
#include "StepSolveCommand.h"
#include "WaitForStart.h"
#include "StepSolveMaze.h"
#include "Finish.h"

StepSolveCommand::StepSolveCommand(Solver *solver) : CommandGroup("Solve") {
  addSequential(new WaitForStart());
  addSequential(new StepSolveMaze(solver));
  addSequential(new ReturnToStart(solver->mouse));
  addSequential(new SpeedRun(solver->mouse));
  addSequential(new ReturnToStart(solver->mouse));
  addSequential(new Finish(solver->mouse->maze));

}
