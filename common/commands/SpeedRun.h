#pragma once

#include <common/commanduino/CommanDuino.h>
#include <common/core/Solver.h>
#include <common/core/Mouse.h>

class SpeedRun : public CommandGroup {
public:
  SpeedRun(Mouse *mouse);

  void initialize();

  bool isFinished();

private:
  Mouse *mouse;
  route_t *path;
  int index;
};
