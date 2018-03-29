#pragma once

#include <common/commanduino/Command.h>
#include <real/RealMouse.h>

class BatteryMonitor : public Command {
 public:
  BatteryMonitor();

  void execute();

  bool isFinished();

 private:
  RealMouse *mouse;
};



