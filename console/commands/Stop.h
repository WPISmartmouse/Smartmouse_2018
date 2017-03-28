#pragma once

#ifdef CONSOLE

#include "CommanDuino.h"
#include "ConsoleMouse.h"
#include "Mouse.h"

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
