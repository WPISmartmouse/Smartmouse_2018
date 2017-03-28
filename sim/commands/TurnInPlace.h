#pragma once
#ifdef SIM

#include <ignition/math.hh>
#include <WallFollower.h>
#include "CommanDuino.h"
#include "SimMouse.h"
#include "Direction.h"

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
