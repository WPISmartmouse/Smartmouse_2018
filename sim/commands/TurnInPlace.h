#pragma once
#ifdef SIM

#include <ignition/math.hh>
#include <common/commanduino/CommanDuino.h>
#include <common/Direction.h>

#include "WallFollower.h"
#include "SimMouse.h"

class TurnInPlace: public Command {
public:
  TurnInPlace(Direction dir);

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:
  double yawDiff(double y1, double y2);
  void setSpeedLimited(double l, double r);

  double goalYaw;
  double dYaw;
  SimMouse *mouse;
  Direction dir;

  const double kP = 0.08;
};

#endif
