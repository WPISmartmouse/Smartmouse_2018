#pragma once

#include <common/commanduino/CommanDuino.h>
#include <common/core/Mouse.h>

#include "RealMouse.h"

class ForwardToCenter : public Command {
public:
  ForwardToCenter();

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:

  GlobalPose start;
  RealMouse *mouse;

  RangeData range_data;
  const double kDisp = 4.0;
};

