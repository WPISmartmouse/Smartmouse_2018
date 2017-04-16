#include "SolveMaze.h"
#include "Forward.h"
#include "Turn.h"
#include "WaitForStart.h"

SolveMaze::SolveMaze(Solver *solver) : CommandGroup("solve"), solver(solver), movements(0),
                                       goal_row(AbstractMaze::MAZE_SIZE / 2), goal_col(AbstractMaze::MAZE_SIZE / 2) {}

SolveMaze::SolveMaze(Solver *solver, int goal_row, int goal_col) : CommandGroup("solve"), solver(solver), movements(0), goal_row(goal_row), goal_col(goal_col) {}

void SolveMaze::initialize() {
  solver->setGoal(goal_row, goal_col);
}

bool SolveMaze::isFinished() {
  bool groupFinished = CommandGroup::isFinished();

  if (groupFinished) {
    bool mazeSolved = solver->isFinished();

    if (!mazeSolved) {
      Direction nextDirection = solver->planNextStep();
      if (nextDirection == solver->mouse->getDir()) {
        addSequential(new Forward());
      }
      else {
        addSequential(new Turn(nextDirection));
      }

      movements++;
    } else {
      return true;
    }

    return false;
  }

  return false;
}

void SolveMaze::end() {
  solver->teardown();
}
