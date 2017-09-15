#pragma once
#include <common/commanduino/CommanDuino.h>
#include "RealMouse.h"
#include <common/core/Direction.h>

class Turn : public CommandGroup {
public:
  Turn(Direction dir);

  void initialize();

private:
  RealMouse *mouse;
  Direction dir;
};

