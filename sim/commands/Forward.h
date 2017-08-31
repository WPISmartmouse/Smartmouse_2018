#pragma once

#include <common/commanduino/CommanDuino.h>
#include <common/Mouse.h>
#include <common/Pose.h>
#include <common/DriveStraight.h>

#include <sim/lib/SimMouse.h>

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

