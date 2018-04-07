#pragma once

#include <common/commanduino/CommanDuino.h>
#include <common/core/Mouse.h>
#include <common/core/Pose.h>

#include <real/RealMouse.h>
#include <common/KinematicController/VelocityProfile.h>

class Backup : public Command {
public:
  Backup();

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:
  RealMouse *mouse;
};

