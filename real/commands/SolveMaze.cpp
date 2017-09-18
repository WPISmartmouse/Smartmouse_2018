#include "SolveMaze.h"
#include "Forward.h"
#include "Turn.h"
#include "ForwardToCenter.h"
#include "TurnInPlace.h"
#include "Stop.h"
#include "ForwardN.h"

SolveMaze::SolveMaze(Solver *solver, Solver::Goal goal) : CommandGroup("solve"), solver(solver), movements(0),
                                                          goal(goal) {}

void SolveMaze::initialize() {
  solved = false;
  atCenter = false;
  solver->setGoal(goal);
}

bool SolveMaze::isFinished() {
  bool groupFinished = CommandGroup::isFinished();

  if (groupFinished) {
    bool mazeSolved = solver->isFinished();

    if (!mazeSolved) {
      motion_primitive_t prim = solver->planNextStep();
      print("%i:%c\r\n", prim.n, dir_to_char(prim.d));

      if (!solver->isSolvable()) {
        solved = false;
        return true;
      }

      if (prim.d == solver->mouse->getDir()) {
        addSequential(new ForwardN(prim.n));
      } else {
        addSequential(new Turn(prim.d));
      }
//      addSequential(new Stop(500));

      movements++;
    } else if (!atCenter){
      addSequential(new ForwardToCenter());
      if (goal == Solver::Goal::START) {
        addSequential(new TurnInPlace(Direction::E));
      }
      atCenter = true;
      solved = true;
      return false;
    } else {
      return true;
    }

    return false;
  }

  return false;
}

void SolveMaze::end() {
  print("solve time (seconds): %lu\r\n", getTime() / 1000ul);
  solver->teardown();
}
