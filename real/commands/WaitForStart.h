#pragma once

#include <common/commanduino/CommanDuino.h>
#include "RealMouse.h"

class WaitForStart : public CommandGroup {
public:
  WaitForStart();
  void initialize();
  void execute();
  bool isFinished();
  void end();

private:
  double speed;
  int init_ticks_left;
  int init_ticks_right;
  static bool calibrated;
  RealMouse *mouse;
};
