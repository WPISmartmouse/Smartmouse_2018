#pragma once

#include <common/commanduino/CommanDuino.h>
#include "RealMouse.h"

class WaitForStart : public CommandGroup {
public:
  WaitForStart();
  bool isFinished();
};
