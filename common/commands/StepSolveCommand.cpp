#include "StepSolveCommand.h"
#include "WaitForStart.h"
#include "StepSolveMaze.h"
#include "Finish.h"

StepSolveCommand::StepSolveCommand(Solver *solver) : CommandGroup("Solve") {
  addSequential(new WaitForStart());
  addSequential(new StepSolveMaze(solver));
  //addSequential(new ReturnToStart());
  //addSequential(new SpeedRun());
  addSequential(new Finish(solver->mouse->maze));

}
