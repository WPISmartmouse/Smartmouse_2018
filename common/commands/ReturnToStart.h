#pragma once

#include <common/commanduino/CommanDuino.h>
#include <common/Solver.h>
#include <common/Mouse.h>

class ReturnToStart : public CommandGroup {
public:
  ReturnToStart(Mouse *mouse);

  void initialize();

  bool isFinished();

private:
  char *pathToStart;
  int index;
  Mouse *mouse;

};
