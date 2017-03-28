#pragma once
#ifdef SIM

#include "CommanDuino.h"
#include "SimMouse.h"
#include "Direction.h"

class Turn : public CommandGroup {
public:
  Turn(Direction dir);
  void initialize();

private:
  SimMouse *mouse;
  Direction dir;
};

#endif
