#pragma once

#include <common/commanduino/CommanDuino.h>
#include <common/core/Mouse.h>
#include <common/core/Pose.h>

#include <sim/lib/SimMouse.h>
#include <common/KinematicController/DriveStraightState.h>

class ForwardN : public Command {
public:
  ForwardN(unsigned int n);

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:

  GlobalPose start;
  SimMouse *mouse;
  unsigned int n;
  smartmouse::kc::DriveStraightState *drive_straight_state;
};

