#pragma once

#include <common/commanduino/CommanDuino.h>
#include <common/Mouse.h>

#include "RealMouse.h"
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
  RealMouse *mouse;

  RangeData range_data;
  WallFollower follower;
  const double kDisp = 0.4;
};

