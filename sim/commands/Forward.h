#pragma once

#ifdef SIM

#include <ignition/math.hh>
#include "CommanDuino.h"
#include "SimMouse.h"
#include "Mouse.h"
#include "WallFollower.h"

class Forward : public Command {
public:
  Forward();

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:

  ignition::math::Pose3d start;
  SimMouse *mouse;

  bool checkedWalls;
  SimMouse::RangeData range_data;
  WallFollower follower;
  bool walls[4];
  bool wallOnLeft, wallOnRight;
};

#endif
