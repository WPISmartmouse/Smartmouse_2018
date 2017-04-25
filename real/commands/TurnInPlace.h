#pragma once
#include <common/commanduino/CommanDuino.h>
#include <common/Direction.h>

#include <common/DriveStraight.h>
#include <real/RealMouse.h>

class TurnInPlace : public Command {
public:
  TurnInPlace(Direction dir);

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:
  double goalYaw;
  double dYaw;
  RealMouse *mouse;
  Direction dir;

  const double kP = 0.21;
};

