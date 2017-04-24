#pragma once

#include <common/Pose.h>
#include <common/Mouse.h>
#include <common/Direction.h>
#include <common/RobotConfig.h>

class WallFollower {
public:
  WallFollower();

  WallFollower(double goalDisp);

  static double dispToNextEdge(Mouse *mouse);

  static double fwdDispToCenter(Mouse *mouse);

  static double forwardDisplacement(Direction dir, Pose start_pose, Pose end_pose);

  static double dispToLeftEdge(Mouse *mouse);

  static double sidewaysDispToCenter(Mouse *mouse);

  static double yawDiff(double y1, double y2);

  static std::pair<double, double> estimate_pose(RangeData range_data, Mouse *mouse);

  std::pair<double, double> compute_wheel_velocities(Mouse *mouse, Pose start_pose, RangeData range_data);

  double disp;
  double goalDisp;
  double dispError;
  static const double kPWall;
  static const double kDWall;
  static const double kPYaw;
};

