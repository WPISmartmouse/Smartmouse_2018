#pragma once

#include <common/commanduino/CommanDuino.h>
#include <common/Mouse.h>
#include <common/Pose.h>

#include <real/RealMouse.h>

class ForwardN : public Command {
public:
  ForwardN(unsigned int n);

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:

  unsigned int n;
  GlobalPose start;
  RealMouse *mouse;

  RangeData range_data;
};

