#pragma once

#include <common/commanduino/CommanDuino.h>
#include <common/Mouse.h>
#include <common/Direction.h>
#include <sim/SimMouse.h>
#include <common/DriveStraight.h>

class ForwardToDiagonal : public Command {
public:
  ForwardToDiagonal();

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:
  GlobalPose start;
  SimMouse *mouse;
  RangeData range_data;
  DriveStraight driver;
  const double kDisp = 1;
};

