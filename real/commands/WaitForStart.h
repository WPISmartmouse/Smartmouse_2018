#pragma once

#include <common/commanduino/CommanDuino.h>
#include "RealMouse.h"

class WaitForStart : public CommandGroup {
public:
  WaitForStart();
  void initialize();
  void execute();
  bool isFinished();
  void end();

private:
  double speed;
  RealMouse *mouse;
};
