#pragma once

#include <common/commanduino/CommanDuino.h>
#include <common/KinematicController/VelocityProfile.h>
#include <sim/lib/SimMouse.h>

class ForwardToCenter : public Command {
public:
  ForwardToCenter();

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:

  GlobalPose start;
  SimMouse *mouse;
  smartmouse::kc::VelocityProfile *profile;
};

