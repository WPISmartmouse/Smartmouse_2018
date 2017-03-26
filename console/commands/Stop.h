#pragma once

#ifdef CONSOLE

#include <common/commanduino/Command.h>
#include "ConsoleMouse.h"

class Stop : public Command {
  public:
    Stop();
    void initialize();
    void execute();
    bool isFinished();
    void end();

  private:
    ConsoleMouse *mouse;
};
#endif
