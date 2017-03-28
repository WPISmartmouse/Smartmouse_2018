#pragma once

#ifdef SIM

#include "SimMouse.h"
#include <ignition/math.hh>

class WallFollower {
public:
  WallFollower();
  WallFollower(double goalDisp);

  static double forwardDisplacement(Direction dir, ignition::math::Pose3d start_pose, ignition::math::Pose3d end_pose);

  static double dispToEdge(SimMouse *mouse);

  static double dispToCenter(SimMouse *mouse);

  static double yawDiff(double y1, double y2);

  std::pair<double, double>
  compute_wheel_velocities(SimMouse *mouse, ignition::math::Pose3d start_pose, SimMouse::RangeData range_data);

  double disp;
  double goalDisp;
  double dispError;
  double angleError;
  double dToWallLeft;
  double dToWallRight;
  const double kPWall = 2;
  const double kDWall = 0;
};

#endif
