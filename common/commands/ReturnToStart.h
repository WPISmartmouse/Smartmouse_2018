#pragma once

#include <common/commanduino/CommanDuino.h>
#include <common/core/Solver.h>
#include <common/core/Mouse.h>

class ReturnToStart : public CommandGroup {
public:
  ReturnToStart(Mouse *mouse);

  void initialize();

  bool isFinished();

private:
  route_t pathToStart;
  int index;
  Mouse *mouse;

};
