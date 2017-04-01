#pragma once

#ifdef SIM

#include <ignition/math.hh>
#include <common/commanduino/CommanDuino.h>
#include <common/Mouse.h>
#include <common/Pose.h>

#include "SimMouse.h"
#include "WallFollower.h"

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

  SimMouse::RangeData range_data;
  WallFollower follower;
};

#endif
