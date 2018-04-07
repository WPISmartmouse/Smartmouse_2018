#include <commands/Turn.h>
#ifndef CONSOLE
#include <commands/ForwardToCenter.h>
#include <commands/TurnInPlace.h>
#endif
#include <common/commands/SolveCommand.h>
#include <commands/WaitForStart.h>
#include <commands/Stop.h>
#include <commands/Finish.h>
#include <commands/SolveMaze.h>

SolveCommand::SolveCommand(Solver *solver) : CommandGroup("SolveGroup"), solver(solver) {}


void SolveCommand::initialize() {
  runs = 0;
  if (!GlobalProgramSettings.quiet) {
    addSequential(new WaitForStart());
  }
  solver->setup();
  addSequential(new Stop(1000));
  addSequential(new SolveMaze(solver, Solver::Goal::CENTER));
  addSequential(new Finish(solver->mouse->maze));
  addSequential(new SolveMaze(solver, Solver::Goal::START));
}

bool SolveCommand::isFinished() {
  bool groupFinished = CommandGroup::isFinished();

  if (groupFinished) {
    runs++;

    // we could check for some finished button here
    if (runs == MAX_RUNS) {
      return true;
    }

    if (!GlobalProgramSettings.quiet) {
      addSequential(new Stop(200));
      addSequential(new WaitForStart());
    }
    addSequential(new SolveMaze(solver, Solver::Goal::CENTER));
    addSequential(new Finish(solver->mouse->maze));
    addSequential(new SolveMaze(solver, Solver::Goal::START));
    return false;
  }

  return false;
}
