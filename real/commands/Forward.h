#pragma once

#include <common/commanduino/CommanDuino.h>
#include <common/core/Mouse.h>
#include <common/core/Pose.h>

#include <real/RealMouse.h>
#include <common/KinematicController/VelocityProfile.h>

class Forward : public Command {
public:
  Forward();

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:
  GlobalPose start;
  RealMouse *mouse;
  smartmouse::kc::VelocityProfile *profile;
};

