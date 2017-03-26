#pragma once

#ifdef SIM

#include <ignition/math.hh>
#include "common/commanduino/Command.h"
#include "sim/SimMouse.h"
#include "common/Mouse.h"

class Stop : public Command {
  public:
    Stop();
    void initialize();
    void execute();
    bool isFinished();
    void end();

  private:
    SimMouse *mouse;
};
#endif
