#pragma once
#include <ignition/math.hh>
#include <common/commanduino/CommanDuino.h>
#include <common/Direction.h>

#include <common/WallFollower.h>
#include "SimMouse.h"

class TurnInPlace : public Command {
public:
  TurnInPlace(Direction dir);

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:
  double limit(double x);

  double goalYaw;
  double dYaw;
  SimMouse *mouse;
  Direction dir;

  const double kP = 0.12;
};

