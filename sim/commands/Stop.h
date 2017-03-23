#pragma once

#ifdef SIM

#include <ignition/math.hh>
#include "CommanDuino.h"
#include "SimMouse.h"
#include "Mouse.h"

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
