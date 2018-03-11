#pragma once
#include <ignition/math.hh>
#include <common/commanduino/CommanDuino.h>
#include <common/core/Direction.h>
#include <sim/lib/SimMouse.h>

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
  SimMouse *mouse;
  Direction dir;

  const static double kP;
};

