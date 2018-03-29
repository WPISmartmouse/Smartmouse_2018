#pragma once

#include <common/commanduino/Command.h>
#include <real/RealMouse.h>

class BatteryMonitor : public Command {
 public:
  BatteryMonitor();

  void Execute();

  bool IsFinished();

 private:
  RealMouse *mouse;
};



