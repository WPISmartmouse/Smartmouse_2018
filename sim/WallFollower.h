#pragma once

#ifdef SIM

#include <common/Pose.h>
#include <ignition/math.hh>

#include "SimMouse.h"

class WallFollower {
public:
  WallFollower();

  WallFollower(double goalDisp);

  static double forwardDisplacement(Direction dir, Pose start_pose, Pose end_pose);

  static double dispToEdge(SimMouse *mouse);

  static double dispToCenter(SimMouse *mouse);

  static double yawDiff(double y1, double y2);

  std::pair<double, double>
  compute_wheel_velocities(SimMouse *mouse, Pose start_pose, SimMouse::RangeData range_data);

  double disp;
  double goalDisp;
  double dispError;
  double angleError;
  double dToWallLeft;
  double dToWallRight;
  double lastLeftWallError;
  double lastRightWallError;
  const double kPWall = 0.2;
  const double kDWall = 80;
};

#endif
