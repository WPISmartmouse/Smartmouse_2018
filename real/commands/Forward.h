#pragma once

#include <common/commanduino/CommanDuino.h>
#include <common/Mouse.h>
#include <common/Pose.h>

#include "RealMouse.h"

class Forward : public Command {
public:
  Forward();

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:
  GlobalPose start;
  RealMouse *mouse;

  RangeData range_data;
};

