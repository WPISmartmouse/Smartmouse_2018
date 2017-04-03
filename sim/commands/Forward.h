#pragma once

#ifdef SIM

#include <ignition/math.hh>
#include <common/commanduino/CommanDuino.h>
#include <common/Mouse.h>
#include <common/Pose.h>
#include <common/WallFollower.h>

#include "SimMouse.h"

class Forward : public Command {
public:
  Forward();

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:

  Pose start;
  SimMouse *mouse;

  RangeData range_data;
  WallFollower follower;
};

#endif
