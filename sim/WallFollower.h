#pragma once

#ifdef SIM

#include "SimMouse.h"
#include <ignition/math.hh>

class WallFollower {
public:
  WallFollower();
  WallFollower(double goalDisp);

  static double forwardDisplacement(Direction dir, ignition::math::Pose3d start_pose, ignition::math::Pose3d end_pose);

  static double dispToEdge(Direction dir, ignition::math::Pose3d current_pose);

  static double yawDiff(double y1, double y2);

  std::pair<double, double>
  compute_wheel_velocities(SimMouse *mouse, ignition::math::Pose3d start_pose, SimMouse::RangeData range_data);

  double disp;
  double goalDisp;
  double dispError;
  double angleError;
  double dToWallLeft;
  double dToWallRight;
  const float kPWall = 4;
  const float FORWARD_SPEED = 4.0;
};

#endif
