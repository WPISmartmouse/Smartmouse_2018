#include "Turn.h"
#ifndef CONSOLE
#include "ForwardToCenter.h"
#include "TurnInPlace.h"
#endif
#include "SolveCommand.h"
#include "WaitForStart.h"
#include "Stop.h"
#include "Finish.h"

SolveCommand::SolveCommand(Solver *solver) : CommandGroup("SolveGroup"), solver(solver) {}


void SolveCommand::initialize() {
  runs = 0;
  if (!GlobalProgramSettings.q) {
    addSequential(new WaitForStart());
  }
  solver->setup();
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

    if (!GlobalProgramSettings.q) {
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
