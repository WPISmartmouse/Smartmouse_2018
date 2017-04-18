#include "Stop.h"
#include "SolveCommand.h"
#include "WaitForStart.h"

SolveCommand::SolveCommand(Solver *solver) : CommandGroup("SolveGroup"), solver(solver) {}


void SolveCommand::initialize() {
  runs = 0;
//  if (!GlobalProgramSettings.q) {
//    addSequential(new WaitForStart());
//  }
  solver->setup();
  addSequential(new Stop(200));
  addSequential(new SolveMaze(solver));
  addSequential(new SolveMaze(solver, 0, 0));
}

bool SolveCommand::isFinished() {
  bool groupFinished = CommandGroup::isFinished();

  if (groupFinished) {
    runs++;

    // we could check for some finished button here
    if (runs == MAX_RUNS) {
      return true;
    }

    to_center = new SolveMaze(solver);
    to_start  = new SolveMaze(solver, 0, 0);
    addSequential(to_center);
    addSequential(to_start);
    return false;
  }

  return false;

}
