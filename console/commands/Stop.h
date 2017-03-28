#pragma once

#ifdef CONSOLE

#include <common/commanduino/CommanDuino.h>
#include <common/Mouse.h>

#include "ConsoleMouse.h"

class Stop : public Command {
public:
  Stop(unsigned long stop_time);

  Stop();

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:
  ConsoleMouse *mouse;
};

#endif
