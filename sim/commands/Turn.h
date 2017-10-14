#pragma once
#include <common/commanduino/CommanDuino.h>
#include <sim/lib/SimMouse.h>
#include <common/core/Direction.h>

class Turn : public CommandGroup {
public:
  Turn(Direction dir);

  void initialize();

private:
  SimMouse *mouse;
  Direction dir;
};

