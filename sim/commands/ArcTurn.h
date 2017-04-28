#pragma once
#include <common/commanduino/CommanDuino.h>
#include <common/Direction.h>

#include <sim/SimMouse.h>

#include <common/RobotConfig.h>
#include <common/AbstractMaze.h>

class ArcTurn : public Command{
public:
  ArcTurn(Direction dir);

  void initialize();

  void execute();

  bool isFinished();

  void end();
private:
  double dYaw;
  double dDisp;
  double goalYaw;
  GlobalPose startPose;
  unsigned int startCol;
  unsigned int startRow;

  double vtc_x;
  double vtc_y;

  double end_x;
  double end_y;

  SimMouse* mouse;
  Direction dir;
  double SLOW_ARC_SPEED = 0.75*(config.MAX_SPEED/(AbstractMaze::HALF_UNIT_DIST+(config.TRACK_WIDTH/2)))*(AbstractMaze::HALF_UNIT_DIST-(config.TRACK_WIDTH/2));
  double FAST_ARC_SPEED = 0.75*config.MAX_SPEED;

  double pose_dist(GlobalPose pose, double x, double y);
};
