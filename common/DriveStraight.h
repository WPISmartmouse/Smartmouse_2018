#pragma once

#include <common/Pose.h>
#include <common/Mouse.h>
#include <common/Direction.h>
#include <common/RobotConfig.h>

class DriveStraight {
public:

  DriveStraight();

  void start(GlobalPose start_pose, double goalDisp);

  static double dispToNextEdge(Mouse *mouse);

  static double fwdDispToCenter(Mouse *mouse);

  static double fwdDispToDiag(Mouse *mouse);

  static double fwdDisp(Direction dir, GlobalPose current_pose, GlobalPose start_pose);

  double sidewaysDispToCenter(Mouse *mouse);

  std::pair<double, double> compute_wheel_velocities(Mouse *mouse);

  double disp;
  double goalDisp;
  double dispError;
  GlobalPose start_pose;
  static const double kPWall;
  static const double kDWall;
  static const double kPYaw;
};

