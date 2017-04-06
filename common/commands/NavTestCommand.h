#pragma once

#include <common/commanduino/CommanDuino.h>
#include <common/Solver.h>
#include <list>

class NavTestCommand : public CommandGroup {
public:
  NavTestCommand(Solver *solver);

  void initialize();

  bool isFinished();

  void end();

private:
  Solver *solver;
  std::list<Command *> commands;
  std::list<Command *>::iterator cmd_it;
};
