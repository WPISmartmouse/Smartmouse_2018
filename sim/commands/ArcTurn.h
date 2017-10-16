//TODO: finish unit conversion

#pragma once
#include <common/commanduino/CommanDuino.h>
#include <common/core/Direction.h>

#include <sim/lib/SimMouse.h>

#include <common/KinematicController/RobotConfig.h>
#include <common/core/AbstractMaze.h>

class ArcTurn : public Command {
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

  SimMouse *mouse;
  Direction dir;
  double SLOW_ARC_SPEED =
      0.75 * (smartmouse::kc::MAX_SPEED_CUPS / (smartmouse::maze::HALF_UNIT_DIST + (smartmouse::kc::TRACK_WIDTH / 2)))
          * (smartmouse::maze::HALF_UNIT_DIST - (smartmouse::kc::TRACK_WIDTH / 2));
  double FAST_ARC_SPEED = 0.75 * smartmouse::kc::MAX_SPEED_CUPS;

  double pose_dist(GlobalPose pose, double x, double y);
};
