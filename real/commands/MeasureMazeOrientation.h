#pragma once

#include <common/commanduino/CommanDuino.h>
#include "RealMouse.h"

class MeasureMazeOrientation : public Command {
public:
  MeasureMazeOrientation();

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:
  RealMouse *mouse;
};
