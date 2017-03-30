#pragma once

#ifdef SIM

#include <ignition/math.hh>
#include <common/commanduino/CommanDuino.h>
#include <common/Mouse.h>

#include "SimMouse.h"
#include "WallFollower.h"

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

  bool checkedWalls;
  SimMouse::RangeData range_data;
  WallFollower follower;
  bool walls[4];
  bool wallOnLeft, wallOnRight;
  const double kDisp = 2;
};

#endif
