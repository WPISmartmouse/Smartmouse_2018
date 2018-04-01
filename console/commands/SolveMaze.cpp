#include "SolveMaze.h"
#include "Turn.h"
#include "WaitForStart.h"
#include "ForwardN.h"

SolveMaze::SolveMaze(Solver *solver, Solver::Goal goal) : CommandGroup("solve"), solver(solver), movements(0),
                                                          goal(goal) {}

void SolveMaze::initialize() {
  solved = false;
  solver->setGoal(goal);
}

bool SolveMaze::isFinished() {
  bool groupFinished = CommandGroup::isFinished();

  if (groupFinished) {
    bool mazeSolved = solver->isFinished();

    if (!mazeSolved) {
      motion_primitive_t prim = solver->planNextStep();

      if (prim.d == Direction ::INVALID || !solver->isSolvable()) {
        solved = false;
        return true;
      }

      addSequential(new Turn(prim.d));
      addSequential(new ForwardN(prim.n));
      if (!GlobalProgramSettings.quiet) {
        addSequential(new WaitForStart());
        solver->mouse->print_maze_mouse();
      }
    } else {
      solved = true;
      return true;
    }

    return false;
  }

  return false;
}

void SolveMaze::end() {
  solver->teardown();
}
