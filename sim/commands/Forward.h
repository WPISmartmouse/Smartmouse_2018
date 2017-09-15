#pragma once

#include <ignition/math.hh>
#include <common/commanduino/CommanDuino.h>
#include <common/core/Mouse.h>
#include <common/core/Pose.h>
#include <common/DriveStraight.h.not>

#include "SimMouse.h"

class Forward : public Command {
public:
  Forward();

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:

  GlobalPose start;
  SimMouse *mouse;

  RangeData range_data;
  DriveStraight follower;
};

