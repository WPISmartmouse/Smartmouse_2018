#pragma once

#include <common/commanduino/CommanDuino.h>
#include "RealMouse.h"

class Calibrate : public Command {
public:
  Calibrate();

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:
  RealMouse *mouse;
};
