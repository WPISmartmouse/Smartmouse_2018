#pragma once

#include <common/commanduino/CommanDuino.h>
#include <common/core/Direction.h>

#include <real/RealMouse.h>

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
  RealMouse *mouse;
  Direction goal_dir, start_dir;

  static const double kP;
};

