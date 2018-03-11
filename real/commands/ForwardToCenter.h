#pragma once

#include <common/commanduino/CommanDuino.h>
#include <common/core/Mouse.h>
#include <common/KinematicController/DriveStraightState.h>

#include <real/RealMouse.h>

class ForwardToCenter : public Command {
public:
  ForwardToCenter();

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:
  GlobalPose start;
  RealMouse *mouse;
  smartmouse::kc::DriveStraightState *drive_straight_state;
};

