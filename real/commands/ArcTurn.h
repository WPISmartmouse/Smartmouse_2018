#pragma once

#include <common/commanduino/CommanDuino.h>
#include <common/Direction.h>

#include <real/RealMouse.h>

#include <common/RobotConfig.h>
#include <common/AbstractMaze.h>

class ArcTurn : public Command {
public:
  ArcTurn(Direction dir);

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:
  RealMouse *mouse;
  Direction dir;

  GlobalPose curPose;
  unsigned int curCol;
  unsigned int curRow;
  Direction curDir;

  GlobalPose startPose;
  unsigned int startCol;
  unsigned int startRow;

  double dYaw;
  double goalYaw;

  double vtc_x;
  double vtc_y;

  constexpr static double speed_scale = 0.75;
  double SLOW_ARC_SPEED = speed_scale * (config.MAX_SPEED / (AbstractMaze::HALF_UNIT_DIST + (config.TRACK_WIDTH / 2))) *
                          (AbstractMaze::HALF_UNIT_DIST - (config.TRACK_WIDTH / 2));
  double FAST_ARC_SPEED = speed_scale * (config.MAX_SPEED);

  constexpr static double kp_turn = 3.00;
  constexpr static double ang_weight = 1.00;
  constexpr static double arc_weight = 1.00;

  // 3.00,1.00,0.75

  double pose_dist(GlobalPose pose, double x, double y);
};
