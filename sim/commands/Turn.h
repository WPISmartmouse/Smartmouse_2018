#pragma once
#ifdef SIM

#include <ignition/math.hh>
#include <WallFollower.h>
#include "CommanDuino.h"
#include "SimMouse.h"
#include "Direction.h"

class Turn : public Command {
public:
  Turn(Direction dir);

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
  ignition::math::Pose3d start;

  double l, r;
  WallFollower follower;
  const double kP = 0.05;

  bool full_180;
  int state;

  static constexpr int STOP = 0;
  static constexpr int TURN = 1;
  static constexpr int GO_TO_EDGE = 2;
  static constexpr int DONE = 3;
};

#endif
