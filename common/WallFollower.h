#pragma once

#include <common/Pose.h>
#include <common/Mouse.h>
#include <common/Direction.h>
#include "RobotConfig.h"

class WallFollower {
public:
  WallFollower(RobotConfig config);

  WallFollower(double goalDisp);

  static double dispToNextEdge(Mouse *mouse);

  static double fwdDispToCenter(Mouse *mouse);

  static double forwardDisplacement(Direction dir, Pose start_pose, Pose end_pose);

  static double sidewayDispToCenter(Mouse *mouse);

  static double yawDiff(double y1, double y2);

  std::pair<double, double> compute_wheel_velocities(Mouse *mouse, Pose start_pose, RangeData range_data);

  double disp;
  double goalDisp;
  double dispError;
  double angleError;
  double dToWallLeft;
  double dToWallRight;
  double lastLeftWallError;
  double lastRightWallError;
  static const double kPWall;
  static const double kDWall;
  static const double kPYaw;

  RobotConfig config;
};

