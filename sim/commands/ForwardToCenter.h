#pragma once

#include <ignition/math.hh>
#include <common/commanduino/CommanDuino.h>
#include <common/Mouse.h>

#include "SimMouse.h"
#include <common/DriveStraight.h>

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
  DriveStraight follower;
  const double kDisp = 4;
};

