#pragma once

#include <common/commanduino/CommanDuino.h>
#include <common/Pose.h>
#include "RealMouse.h"

class Forward : public CommandGroup {
public:
  Forward();

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:
  RealMouse *mouse;
};
