#include "Turn.h"
#ifndef CONSOLE
#include "ForwardToCenter.h"
#include "TurnInPlace.h"
#endif
#include "Stop.h"
#include "SolveCommand.h"
#include "WaitForStart.h"

SolveCommand::SolveCommand(Solver *solver) : CommandGroup("SolveGroup"), solver(solver) {}


void SolveCommand::initialize() {
  runs = 0;
  if (!GlobalProgramSettings.q) {
    addSequential(new WaitForStart());
  }
  solver->setup();
  addSequential(new SolveMaze(solver));
  addSequential(new SolveMaze(solver, 0, 0));
#ifndef CONSOLE
  addSequential(new ForwardToCenter());
  addSequential(new TurnInPlace(Direction::E));
#endif
}

bool SolveCommand::isFinished() {
  bool groupFinished = CommandGroup::isFinished();

  if (groupFinished) {
    runs++;

    // we could check for some finished button here
    if (runs == MAX_RUNS) {
      return true;
    }

    if (!GlobalProgramSettings.q) {
      addSequential(new WaitForStart());
    }
    to_center = new SolveMaze(solver);
    to_start  = new SolveMaze(solver, 0, 0);
    addSequential(to_center);
    addSequential(to_start);
#ifndef CONSOLE
    addSequential(new ForwardToCenter());
    addSequential(new TurnInPlace(Direction::E));
#endif
    return false;
  }

  return false;

}
