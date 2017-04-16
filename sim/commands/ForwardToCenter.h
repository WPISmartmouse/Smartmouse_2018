#pragma once

#include <ignition/math.hh>
#include <common/commanduino/CommanDuino.h>
#include <common/Mouse.h>

#include "SimMouse.h"
#include <common/WallFollower.h>

class ForwardToCenter : public Command {
public:
  ForwardToCenter();

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:

  Pose start;
  SimMouse *mouse;

  RangeData range_data;
  WallFollower follower;
  const double kDisp = 0.4;
};

