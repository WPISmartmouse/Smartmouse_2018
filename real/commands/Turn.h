#pragma once

#include <common/commanduino/CommanDuino.h>
#include "RealMouse.h"
#include <common/Direction.h>

class Turn : public Command {
public:
  Turn(Direction dir);

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:
  RealMouse *mouse;
  Direction dir;
};
