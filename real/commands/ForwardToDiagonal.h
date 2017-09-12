#pragma once

#include <common/commanduino/CommanDuino.h>
#include <common/Mouse.h>
#include <common/Direction.h>
#include <real/RealMouse.h>

class ForwardToDiagonal : public Command {
public:
  ForwardToDiagonal();

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:
  GlobalPose start;
  RealMouse *mouse;
  RangeData range_data;
  const double kDisp = 3;
};

