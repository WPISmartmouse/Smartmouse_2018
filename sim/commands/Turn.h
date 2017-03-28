#pragma once
#ifdef SIM

#include <common/commanduino/CommanDuino.h>
#include "SimMouse.h"
#include <common/Direction.h>

class Turn : public CommandGroup {
public:
  Turn(Direction dir);

  void initialize();

private:
  SimMouse *mouse;
  Direction dir;
};

#endif
