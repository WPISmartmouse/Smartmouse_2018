#pragma once
#include <ignition/math.hh>
#include <common/commanduino/CommanDuino.h>
#include <common/Direction.h>

#include <common/WallFollower.h>
#include "SimMouse.h"

class AlignYaw : public Command {
public:
  AlignYaw();

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:
  double limit(double x);

  SimMouse *mouse;
  RangeData range_data;

  const double kP = 0.1;
};

