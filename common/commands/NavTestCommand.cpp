#include "NavTestCommand.h"
#include "WaitForStart.h"
#include "Forward.h"
#include "Stop.h"
#include "Turn.h"

NavTestCommand::NavTestCommand(Solver *solver) : CommandGroup("NavTestGroup"), solver(solver) {
  commands.push_back(new Forward());
  commands.push_back(new Turn(Direction::S));
  commands.push_back(new Stop(100));
  cmd_it = commands.begin();
}

void NavTestCommand::initialize() {
  solver->setup();
}

bool NavTestCommand::isFinished() {
  bool groupFinished = CommandGroup::isFinished();

  if (groupFinished) {
    solver->isFinished();

    if (cmd_it != commands.end()) {
      // update maze, but ignore the planned result
      solver->planNextStep();
      addSequential(*cmd_it);
      cmd_it++;
#ifdef CONSOLE
      addSequential(new WaitForStart());
      solver->mouse->print_maze_mouse();
#endif
    } else {
      return true;
    }

    return false;
  }

  return false;
}

void NavTestCommand::end() {
  solver->teardown();
}
