#pragma once

#include <ignition/math.hh>
#include <common/commanduino/CommanDuino.h>
#include <common/Direction.h>
#include <common/DriveStraight.h>

#include "SimMouse.h"

class DumbArcTurn : public Command {
public:
  DumbArcTurn(Direction dir);

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:
  double goalYaw;
  double dYaw;
  SimMouse *mouse;
  Direction goal_dir, start_dir;

  static const double kP;
};

