#pragma once

#include <common/Pose.h>
#include <common/Mouse.h>
#include <common/Direction.h>
#include "RobotConfig.h"

class WallFollower {
public:
  WallFollower(RobotConfig config);

  WallFollower(double goalDisp);

  static double forwardDisplacement(Direction dir, Pose start_pose, Pose end_pose);

  static double dispToEdge(Mouse *mouse);

  static double dispToCenter(Mouse *mouse);

  static double yawDiff(double y1, double y2);

  std::pair<double, double>
  compute_wheel_velocities(Mouse *mouse, Pose start_pose, RangeData range_data);

  double disp;
  double goalDisp;
  double dispError;
  double angleError;
  double dToWallLeft;
  double dToWallRight;
  double lastLeftWallError;
  double lastRightWallError;
  const double kPWall = 0.1;
  const double kDWall = 50;
  const double kPYaw = 2;

  RobotConfig config;
};

