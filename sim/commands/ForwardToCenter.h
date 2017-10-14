#pragma once

#include <common/commanduino/CommanDuino.h>

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
};

