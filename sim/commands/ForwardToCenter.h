#pragma once

#include <common/commanduino/CommanDuino.h>

#include "SimMouse.h"
#include <common/DriveStraight.h.not>

class ForwardToCenter : public Command {
public:
  ForwardToCenter();

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:

  GlobalPose start;
  SimMouse *mouse;

  RangeData range_data;
  DriveStraight driver;
  const double kDisp = 4;
};

