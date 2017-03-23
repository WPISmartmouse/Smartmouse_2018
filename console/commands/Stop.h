#pragma once

#ifdef CONSOLE

#include "CommanDuino.h"
#include "ConsoleMouse.h"
#include "Mouse.h"

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
