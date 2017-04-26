#pragma once

#include <common/commanduino/CommanDuino.h>
#include <common/Direction.h>
#include <common/DriveStraight.h>

#include <real/RealMouse.h>

class ArcTurn : public Command {
public:
  ArcTurn(Direction dir);

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

